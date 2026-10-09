/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// Required stub for bare-metal C++ pure virtual functions
extern "C" void __cxa_pure_virtual() { while (1); }

#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "leafbms.h"
#include "chademo.h"
#include "linbus.h"
#include "picontroller.h"
#include "my_string.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static Can* can;
static LinBus* lin;
static PiController fuelGaugeController;
int g_canrun = 0;
bool g_shifterChanged = false;
static float g_vehicleSpeedKmh = 0; //from SKID_ECU 0x0B4, real Toyota ABS speed - not the vestigial VAG wheelfl/wheelfr traction control path

//Values of byte 1 in the 0x540 shift lever message
enum gears
{
   GEAR_B = 0x00,
   GEAR_D = 0x10,
   GEAR_N = 0x20,
   GEAR_R = 0x40,
   GEAR_P = 0x80
};

//P1 park button on J2-54 (SIG_FORWARD, 3k3 series into PA4/ADC4). The switch ladder
//(4.5k released, 0.66k pressed) goes to GND, and the adapter has a pull-up to J2-56 (+12V,
//18k) or J2-48 (+5V, 3.3k). ADC: released ~2200-3600 (12V: depends on battery voltage),
//pressed ~400-1030, open circuit 4095, shorted switch ~0.
//Pressed = above P1_SHORT_MAX and below p1thresh. A short is a wiring fault, not a press.
#define P1_SHORT_MAX      150
#define P1_DEBOUNCE     3    //x 10ms
#define P1_MAX_SPEED      50   //rpm, park only accepted when standing still
#define GEAR_PERIOD       100  //x 10ms, OEM repeats 0x540 about once per second
//No PPOS feedback yet, so approximate park-lock completion with a fixed delay instead
//of real confirmation. Real ECU (20261005 capture): 0x540 shows byte1=0x00 while the pawl
//moves, 0.1s fast release, 0.3s engage, 0.5s first/loaded release, then the new gear lands
//on 0x540 and 0x120 together. BB07D7760 manifest: up to 0.6s from command to complete.
#define PARK_LOCK_DELAY   60   //x 10ms = 0.6s

static uint8_t g_gear = GEAR_P; //car powers up in park
static uint8_t g_reportedGear = GEAR_P; //what 0x540/0x120 broadcast - lags g_gear during a P/not-P transition
static bool g_parkTransit = false; //pawl moving: 0x540 reports byte1=0x00, 0x120 keeps the old gear


static void ProcessCruiseControlButtons()
{
   static bool transition = false;
   static int cruiseTarget = 0;
   int cruisespeed = Param::GetInt(Param::cruisespeed);
   int cruisestt = Param::GetInt(Param::cruisestt);

   if (transition)
   {
      if ((cruisestt & (CRUISE_SETP | CRUISE_SETN)) == 0)
      {
         transition = false;
      }
      return;
   }
   else
   {
      if (cruisestt & (CRUISE_SETP | CRUISE_SETN))
      {
         transition = true;
      }
   }

   if (cruisestt & CRUISE_ON && Param::GetInt(Param::opmode) == MOD_RUN)
   {
      if (cruisespeed <= 0)
      {
         int currentSpeed = Param::GetInt(Param::speed);

         if (cruisestt & CRUISE_SETN && currentSpeed > 500) //Start cruise control at current speed
         {
            cruiseTarget = currentSpeed;
            cruisespeed = cruiseTarget;
         }
         else if (cruisestt & CRUISE_SETP && cruiseTarget > 0) //resume via ramp
         {
            cruisespeed = currentSpeed;
         }
      }
      else
      {
         if (cruisestt & CRUISE_DISABLE || Param::GetBool(Param::din_brake))
         {
            cruisespeed = 0;
         }
         else if (cruisestt & CRUISE_SETP)
         {
            cruiseTarget += Param::GetInt(Param::cruisestep);
         }
         else if (cruisestt & CRUISE_SETN)
         {
            cruiseTarget -= Param::GetInt(Param::cruisestep);
         }
      }
   }
   else
   {
      int regenLevel = Param::GetInt(Param::regenlevel);
      if (cruisestt & CRUISE_SETP)
      {
         regenLevel++;
         regenLevel = MIN(3, regenLevel);
      }
      else if (cruisestt & CRUISE_SETN)
      {
         regenLevel--;
         regenLevel = MAX(0, regenLevel);
      }

      Param::SetInt(Param::regenlevel, regenLevel);
      cruisespeed = 0;
      cruiseTarget = 0;
   }

   if (cruisespeed <= 0)
   {
      Param::SetInt(Param::cruisespeed, 0);
   }
   else if (cruisespeed < cruiseTarget)
   {
      Param::SetInt(Param::cruisespeed, RAMPUP(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiserampup)));
   }
   else if (cruisespeed > cruiseTarget)
   {
      Param::SetInt(Param::cruisespeed, RAMPDOWN(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiserampdn)));
   }
   else
   {
      Param::SetInt(Param::cruisespeed, cruisespeed);
   }
}

static void RunChaDeMo()
{

   if (Param::GetInt(Param::invmode) == INVMOD_CHARGE)
   {
      chargeMode = true;
      Param::SetInt(Param::opmode, MOD_CHARGESTART);
   }

   /* 1s after entering charge mode, enable charge permission */
   if (Param::GetInt(Param::opmode) == MOD_CHARGESTART && rtc_get_counter_val() > 200)
   {
      ChaDeMo::SetEnabled(true);
      ChaDeMo::SetContactor(true);
      Param::SetInt(Param::opmode, MOD_CHARGE);
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGE)
   {
      int chargeCur = Param::GetInt(Param::chgcurlim) * 2;
      int chargeLim = Param::GetInt(Param::chargelimit);
      chargeCur = MIN(MIN(255, chargeLim), chargeCur);
      ChaDeMo::SetChargeCurrent(chargeCur);

      if (Param::GetBool(Param::cdmcheckena))
         ChaDeMo::CheckSensorDeviation(Param::GetInt(Param::udcbms));
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGEND)
   {
      ChaDeMo::SetChargeCurrent(0);
   }

   ChaDeMo::SetTargetBatteryVoltage(Param::GetInt(Param::udclimit));
   ChaDeMo::SetSoC(Param::Get(Param::soc));
   Param::SetInt(Param::cdmcureq, ChaDeMo::GetRampedCurrentRequest());

   if (chargeMode)
   {
      if (Param::GetInt(Param::batfull) ||
          Param::Get(Param::soc) >= Param::Get(Param::soclimit) ||
          Param::GetInt(Param::chargelimit) == 0 ||
          !LeafBMS::Alive(rtc_get_counter_val()))
      {
         if (!LeafBMS::Alive(rtc_get_counter_val()))
         {
            ChaDeMo::SetGeneralFault();
         }
         ChaDeMo::SetEnabled(false);
         Param::SetInt(Param::opmode, MOD_CHARGEND);
      }

      Param::SetInt(Param::udccdm, ChaDeMo::GetChargerOutputVoltage());
      Param::SetInt(Param::idccdm, ChaDeMo::GetChargerOutputCurrent());
      //ChaDeMo::SendMessages(can);
   }
   Param::SetInt(Param::cdmstatus, ChaDeMo::GetChargerStatus());
   if (!LeafBMS::Alive(rtc_get_counter_val()))
   {
      ErrorMessage::Post(ERR_BMSCOMM);
   }
}

static void SendVAG100msMessage()
{
   static int seqCtr = 0;
   static uint8_t ctr = 0;

   const uint8_t seq1[] = { 0x0f, 0x28, 0x7f, 0x28 };
   const uint8_t seq2[] = { 0x1e, 0x10, 0x00, 0x10 };
   const uint8_t seq3[] = { 0x70, 0x56, 0xf0, 0x56 };
   const uint8_t seq4[] = { 0x0c, 0x48, 0xa7, 0x48 };
   const uint8_t seq5[] = { 0x46, 0x90, 0x28, 0x90 };

   uint8_t canData[8] = { (uint8_t)(0x80 | ctr), 0, 0, seq1[seqCtr], seq2[seqCtr], seq3[seqCtr], seq4[seqCtr], seq5[seqCtr] };

   //can->Send(0x580, (uint32_t*)canData);
   seqCtr = (seqCtr + 1) & 0x3;
   ctr = (ctr + 1) & 0xF;
}

static void SetFuelGauge()
{
   static int startupDelay = 500;
   int counts = Param::GetInt(Param::gaugefrq);
   int fuelPos = Param::GetInt(Param::fuelpos);
   int fuelMax = Param::GetInt(Param::gaugemax);
   int soctest = Param::GetInt(Param::soctest);
   int soc = soctest > 0 ? soctest : Param::GetInt(Param::soc);
   int targetPos = (fuelMax * soc) / 100;

   fuelGaugeController.SetRef(FP_FROMINT(fuelPos));
   int dc = fuelGaugeController.Run(FP_FROMINT(targetPos));
   dc *= counts;
   dc /= 1000;

   Param::SetInt(Param::tmphs, dc);

   timer_set_period(FUELGAUGE_TIMER, counts);
   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, dc);
   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, dc);
}

static void RunLin()
{
   static int state = 0;
   int compCmd = Param::GetInt(Param::compressor);
   uint8_t data[8] = { 0xb2, 0x00, 0x00, 0x90, 0xff, 0x00, 0x00, 0x00 };
   //b[1] = 0x05 -> 1kw,  b[1] = 0x12 -> 2kw, b[1] = 0x16 -> 3kw

   if (compCmd > 0)
   {
      data[0] = 0xb3;
      data[1] = compCmd;
   }

   if (lin->HasReceived(33, 8))
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::udcompressor, data[7] * 2);
   }

   switch (state)
   {
   case 0:
      lin->Request(17, 0, 0);
      break;
   case 1:
      lin->Request(33, 0, 0);
      break;
   case 2:
      lin->Request(35, 0, 0);
      break;
   case 3:
      lin->Request(38, 0, 0);
      break;
   case 4:
      memset32((int*)data, 0, 2);
      data[0] = 1;
      lin->Request(32, data, 8);
      break;
   case 5:
      lin->Request(59, data, 8);
      break;
   }

   state = (state + 1) % 6;
}


static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

   LeafBMS::RequestNextFrame(can);
   //LeafBMS::Send100msMessages(can);

   if (!LeafBMS::Alive(rtc_get_counter_val()))
   {
      Param::SetInt(Param::chgcurlim, 0);
      Param::SetInt(Param::chglim, 0);
   }

   ProcessCruiseControlButtons();
   RunChaDeMo();
   RunLin();

   bool start = Param::GetInt(Param::invmode) == MOD_OFF;
   start &= Param::GetInt(Param::udcinv) >= (Param::GetInt(Param::udcbms) - Param::GetInt(Param::bmsinvdiff));

   Param::SetInt(Param::din_start, start);

   //if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      //can->SendAll();
   //SendVAG100msMessage();
}

static void ReadDirectionButtons()
{
   int drivesel = AnaIn::drivesel.Get();

   //Forward button
   if (drivesel > 2500 && drivesel < 3500)
   {
      drivesel = DIR_FORWARD;
   }
   //Reverse button
   else if (drivesel > 1500 && drivesel < 2400)
   {
      drivesel = DIR_REVERSE;
   }
   //Neutral button
   else if (drivesel > 500 && drivesel < 1400)
   {
      drivesel = DIR_NEUTRAL;
   }
   else
   {
      drivesel = DIR_NONE;
   }

   Param::SetInt(Param::drivesel, drivesel);
}

static void ReadShifLever(uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;
   int drivesel = AnaIn::drivesel.Get();
   //int shiftLever = data[0] >> 8;
   int shiftLever = bytes[1];
   //int pot1 = AnaIn::throttle1.Get();
   //int pot2 = AnaIn::throttle2.Get();
   //if (bytes[0] == 0x25) //not in change 0xA5
   //{
     //Forward
     //if (shiftLever == 0x10 || pot1 > 4400) 
     //if (shiftLever == 0x10) 
     //{
     //   drivesel = DIR_FORWARD;
     //}
     //Reverse
     //else if (shiftLever == 0x40 || pot1 < 1200)
     //else if (shiftLever == 0x40)
     //{
     //   drivesel = DIR_REVERSE;
     //}
     //Neutral or park
     //else if ((shiftLever == 0x20 && (pot1 < 4400 && pot1 > 1200)) || shiftLever == 0x80)
     //else if ((shiftLever == 0x20 || shiftLever == 0x80))
     if (shiftLever == 0x80)
     {
        drivesel = DIR_NEUTRAL;
       Param::SetInt(Param::drivesel, drivesel);
     }
     //Param::SetInt(Param::drivesel, drivesel);
   //}
}

static void ReadBrakePedal(uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;
   int brakePedal = bytes[0];
   int brake = 0;
   if (brakePedal == 0x04)
   {
      brake = 1;
   }
   else if (brakePedal == 0x84)
   {
      brake = 0;
   }
   Param::SetInt(Param::din_brake, brake);
}

static void GetDigInputs()
{
   static int lastDrivesel = DIR_NONE;
   int drivesel = Param::GetInt(Param::drivesel);
   int invdir = Param::GetInt(Param::invdir);
   int speed = Param::GetInt(Param::speed);
   int cruisemode = Param::GetInt(Param::cruisestt) & CRUISE_ON;
   int canio = 0;
   bool brakeActive = Param::GetBool(Param::din_brake) || DigIo::brake_in.Get();

   //Forward button
   if (drivesel == DIR_FORWARD)
   {
      //while driving use as cruise control/regen adjust
      if (invdir == DIR_FORWARD && speed > 500)
      {
         Param::SetInt(Param::cruisestt, cruisemode | CRUISE_SETP);
      }
      else if (brakeActive || invdir == DIR_REVERSE)
      {
         Param::SetInt(Param::din_reverse, 0);
         Param::SetInt(Param::din_forward, 1);
      }
   }
   //Reverse button
   else if (drivesel == DIR_REVERSE)
   {
      //while driving use as cruise control/regen adjust
      if (invdir == DIR_FORWARD && speed > 500)
      {
         Param::SetInt(Param::cruisestt, cruisemode | CRUISE_SETN);
      }
      else if (brakeActive || invdir == DIR_FORWARD)
      {
         Param::SetInt(Param::din_forward, 0);
         Param::SetInt(Param::din_reverse, 1);
      }
   }
   //Neutral button
   else if (drivesel == DIR_NEUTRAL)
   {
      if (speed == 0)
      {
         Param::SetInt(Param::din_forward, 1);
         Param::SetInt(Param::din_reverse, 1);
         Param::SetInt(Param::din_bms, 1); //prevents going into charge mode
      }
      /*else if (speed > 500 && lastDrivesel == DIR_NONE)
      {
         if (cruisemode)
            Param::SetInt(Param::cruisestt, 0);
         else
            Param::SetInt(Param::cruisestt, CRUISE_ON);
      }*/
   }
   //No button
   else
   {
      Param::SetInt(Param::din_forward, 0);
      Param::SetInt(Param::din_reverse, 0);
      Param::SetInt(Param::cruisestt, cruisemode);
      Param::SetInt(Param::din_bms, invdir == DIR_REVERSE); //limit torque in reverse
   }

   lastDrivesel = drivesel;

   if (Param::GetBool(Param::din_cruise))
      canio |= CAN_IO_CRUISE;
   if (Param::GetBool(Param::din_start) || DigIo::start_in.Get())
      canio |= CAN_IO_START;
   // Brake bit not forwarded to stm32-sine: Prius 0x030 brake signal is
// always-on and would permanently lock stm32-sine into regen-only mode.
// din_brake is still used internally for the direction change condition.
   if (Param::GetBool(Param::din_forward))
      canio |= CAN_IO_FWD;
   if (Param::GetBool(Param::din_reverse))
      canio |= CAN_IO_REV;
   if (Param::GetBool(Param::din_bms))
      canio |= CAN_IO_BMS;

   Param::SetInt(Param::canio, canio);
}

static void TractionControl(float& throtmin, float& throtmax)
{
   if (!Param::GetBool(Param::espoff))
   {
      float frontAxleSpeed = (Param::GetFloat(Param::wheelfl) + Param::GetFloat(Param::wheelfr)) / 2;
      float rearAxleSpeed = (Param::GetFloat(Param::wheelrl) + Param::GetFloat(Param::wheelrr)) / 2;
      float diff = frontAxleSpeed - rearAxleSpeed;
      float kp = Param::GetFloat(Param::tractionkp);

      //Here we assume front wheel drive
      if (diff < 0)
      {
         //Front axle turns slower than rear axle -> too much breaking force
         float speedErr = Param::GetFloat(Param::allowedlag) - diff;
         throtmin = -100 + kp * speedErr;
      }
      else
      {
         //Front axle turns faster than rear axle -> wheel spin
         float speedErr = Param::GetFloat(Param::allowedspin) - diff;
         throtmax = 100 + kp * speedErr;
      }
   }
}

static void ProcessThrottle()
{
   int pot1 = AnaIn::throttle1.Get();
   int pot2 = AnaIn::throttle2.Get();
   int brakePressure = Param::GetInt(Param::brakepressure);
   int offPedalRegen = Param::GetInt(Param::regenlevel) * 60;

   brakePressure = MAX(offPedalRegen, brakePressure);
   brakePressure = MIN(255, brakePressure);

   /* hard coded throttle redundancy */
   if (pot2 > 50)
   {
      pot1 = MIN(pot1, pot2 * 2);

      if (ABS(2 * pot2 - pot1) > 200 && pot1 < 4090)
         Param::SetInt(Param::errlights, 4);
   }

   Param::SetInt(Param::pot, pot1);
   Param::SetInt(Param::pot2, pot2);
   Param::SetInt(Param::potbrake, brakePressure);
}

static void ProcessShifLever()
{
   int drivesel = Param::GetInt(Param::drivesel);
   int vsx1 = AnaIn::throttle1.Get();
   int vsx3 = AnaIn::throttle2.Get();
   int thresh = Param::GetInt(Param::vsx3thresh);
   int fwdThresh = Param::GetInt(Param::vsx1fwd);
   int revThresh = Param::GetInt(Param::vsx1rev);
   // shiftertype=0 (VSX3ActiveHigh): VSX3 HIGH = lever pushed (LHD, or RHD at 12V)
   // shiftertype=1 (VSX3ActiveLow):  VSX3 LOW  = lever pushed (RHD at 14V/running)
   bool leverActive = (Param::GetInt(Param::shiftertype) == 0) ? (vsx3 > thresh) : (vsx3 < thresh);
   // vsx1fwd/vsx1rev name the raw ADC bands, not the resulting direction - confirmed
   // 2026-09-28 that vsx1 > vsx1rev corresponds to the D gesture and vsx1 < vsx1fwd to R

   if (leverActive)
   {
      if (!g_shifterChanged)
      {
         if (vsx1 > revThresh)
         {
            drivesel = DIR_FORWARD;
            g_shifterChanged = true;
         }
         else if (vsx1 < fwdThresh)
         {
            drivesel = DIR_REVERSE;
            g_shifterChanged = true;
         }
         else
         {
            drivesel = DIR_NEUTRAL;
            // don't latch neutral — keep sampling in case signal is still settling
         }
         Param::SetInt(Param::drivesel, drivesel);

         if (drivesel == DIR_REVERSE)
            g_gear = GEAR_R;
         else if (drivesel == DIR_FORWARD)
            g_gear = GEAR_D;
         else
            g_gear = GEAR_N;
      }
   }
   else
   {
      g_shifterChanged = false;
   }
}

static void ProcessParkButton()
{
   static int pressedCtr = 0;
   static bool wasPressed = false;
   int p1 = AnaIn::p1.Get();
   bool pressed = p1 > P1_SHORT_MAX && p1 < Param::GetInt(Param::p1thresh);

   Param::SetInt(Param::p1, p1);
   pressedCtr = pressed ? MIN(pressedCtr + 1, P1_DEBOUNCE) : 0;
   bool debounced = pressedCtr >= P1_DEBOUNCE;

   //Act on the press only, and only when standing still
   if (debounced && !wasPressed && ABS(Param::GetInt(Param::speed)) <= P1_MAX_SPEED)
   {
      g_gear = GEAR_P;
      Param::SetInt(Param::drivesel, DIR_NEUTRAL);
   }
   wasPressed = debounced;
}

//Drives PCON (TIM1_CH2N) to command the park-lock actuator, and holds g_reportedGear
//at its pre-transition value for PARK_LOCK_DELAY on any P/not-P change - approximating
//the real HV ECU's "gate the reported gear on park-lock feedback" behaviour (BB07D7760
//capture) without real PPOS feedback. While held, SendGearMessage() shows 0x00 on 0x540.
//Non-park gear changes (R/N/D) report immediately.
static void ProcessParkActuator()
{
   static bool lastWantPark = true; //car powers up in park
   static int delayCtr = 0;
   bool wantPark = g_gear == GEAR_P;

   if (wantPark != lastWantPark)
   {
      timer_set_oc_value(TIM1, TIM_OC2, wantPark ? PCON_ENGAGE : PCON_RELEASE);
      lastWantPark = wantPark;
      delayCtr = PARK_LOCK_DELAY;
      g_parkTransit = true;
   }

   if (delayCtr > 0)
   {
      delayCtr--;
   }
   else
   {
      g_parkTransit = false;
      g_reportedGear = g_gear;
   }
}

//Replaces the HV ECU's 0x540 shift lever message. Sent on change and then periodically.
//Byte 0: 0x25 steady, 0xA5 on change. Byte 1: gear. Bytes 2-3: 0.
//Real sequence per P/not-P change (20261005 capture): A5 00 once when the pawl starts
//moving, A5 <gear> when it is done, 25 <gear> 1.0s later and then every 1.0s.
static void SendGearMessage()
{
   static uint8_t lastGear = 0xFF;
   static bool lastTransit = false;
   static int ctr = 0;
   bool changed = g_reportedGear != lastGear;

   Param::SetInt(Param::gear, g_reportedGear);

   if (g_parkTransit && !lastTransit)
   {
      uint8_t data[8] = { 0xA5, GEAR_B, 0, 0, 0, 0, 0, 0 };
      can->Send(0x540, data, 4);
      ctr = 0;
   }
   lastTransit = g_parkTransit;

   if (g_parkTransit)
      return; //stay silent until the pawl is done

   if (changed || ++ctr >= GEAR_PERIOD)
   {
      uint8_t data[8] = { (uint8_t)(changed ? 0xA5 : 0x25), g_reportedGear, 0, 0, 0, 0, 0, 0 };
      can->Send(0x540, data, 4);
      lastGear = g_reportedGear;
      ctr = 0;
   }
}

//Replaces the HV ECU's 0x120 drive mode message (OEM 16.4 ms, we send every 16 ms from Ms16Task).
//Byte 4: 0x10 throughout on a healthy READY car (candump-bb07d77670-...-power-on.csv), but
//the real ECU drops it to 0x00 after ~1.7s when it never reaches READY (20261005 capture,
//unplugged battery/engine ECU captures). Meaning unknown, so we keep sending 0x10.
//Byte 5: gear (0x20 P, 0x21 R, 0x22 N, 0x23 D), byte 6: 0x04 powered / 0x00 standby,
//byte 7: Toyota checksum = sum(bytes 0..6) + length + both ID bytes.
static void SendDriveModeMessage()
{
   uint8_t gearCode;

   switch (g_reportedGear)
   {
   case GEAR_P: gearCode = 0x20; break;
   case GEAR_R: gearCode = 0x21; break;
   case GEAR_N: gearCode = 0x22; break;
   default:     gearCode = 0x23; break; //D and B
   }

   uint8_t data[8] = { 0, 0, 0, 0, 0x10, gearCode, 0, 0 };
   data[6] = Param::GetInt(Param::opmode) == MOD_RUN ? 0x04 : 0x00;

   uint8_t sum = 8 + 0x01 + 0x20;
   for (int i = 0; i < 7; i++)
      sum += data[i];
   data[7] = sum;

   can->Send(0x120, data, 8);
}

//Replaces the HV ECU's 0x3CA speed message (OEM ~100 ms). Byte 2: speed in km/h
//(0-255, unsigned magnitude - direction comes from gear, not this message).
//Byte 4: Toyota checksum = sum(bytes 0..3) + length + both ID bytes.
static void SendSpeedMessage()
{
   static int ctr = 0;

   if (++ctr < 10)
      return;
   ctr = 0;

   uint8_t speed = (uint8_t)MIN(255, ABS(g_vehicleSpeedKmh));

   uint8_t data[5] = { 0, 0, speed, 0, 0 };

   uint8_t sum = 5 + 0x03 + 0xCA;
   for (int i = 0; i < 4; i++)
      sum += data[i];
   data[4] = sum;

   can->Send(0x3CA, data, 5);
}

static void LimitThrottle()
{
   float throtmin = -100, throtmax = 100;

   TractionControl(throtmin, throtmax);

   throtmin = MIN(0, throtmin);
   throtmin = MAX(-100, throtmin);
   throtmax = MIN(100, throtmax);
   throtmax = MAX(0, throtmax);

   Param::SetFloat(Param::calcthrotmax, throtmax);
   Param::SetFloat(Param::calcthrotmin, throtmin);
}

static void SimulateOilSensor()
{
   static int ctr = 0;
   static int state = 0;

   switch (state)
   {
   case 0:
      DigIo::oilevel_out.Set();
      ctr = 2;
      state++;
      break;
   case 1:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Clear();
         ctr = 3;
         state++;
      }
      ctr--;
      break;
   case 2:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Set();
         ctr = 30;
         state++;
      }
      ctr--;
      break;
   case 3:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Clear();
         state = 0;
      }
      ctr--;
      break;
   }
}

static void SetRevCounter()
{
   int regenLevel = Param::GetInt(Param::regenlevel);

   if (Param::GetInt(Param::invmode) != MOD_RUN || Param::GetInt(Param::invdir) == DIR_NEUTRAL)
   {
      DigIo::oilpres_out.Clear();
      Param::SetInt(Param::speedmod, 0);
   }
   else
   {
      DigIo::oilpres_out.Set();
      Param::SetInt(Param::speedmod, 1000 + regenLevel * 1000 - Param::GetInt(Param::idc) * 10);
   }
}

//0x120 is sent by the real ECU every 16.4 ms, which the 10 ms task can't hit
static void Ms16Task(void)
{
   SendDriveModeMessage();
}

static void Ms10Task(void)
{
   const uint8_t seq2[] = { 0x10, 0x68, 0x94, 0xC0 };
   static int seq1Ctr = 0;
   static uint16_t consumptionCounter = 0;
   static uint32_t accumulatedRegen = 0;
   int vacuumthresh = Param::GetInt(Param::vacuumthresh);
   int vacuumhyst = Param::GetInt(Param::vacuumhyst);
   int vacuum = AnaIn::vacuum.Get();
   int invmode = Param::GetInt(Param::invmode);
   int cruiselight = Param::GetInt(Param::cruiselight);
   int errlights = Param::GetInt(Param::errlights);
   float idc = Param::GetFloat(Param::idc);
   float udcbms = Param::GetFloat(Param::udcbms);
   float power = (idc * udcbms) / 1000.0f;
   float dcdcVoltage = 0;
   int32_t consumptionIncrement = -power * 2.8f;

   seq1Ctr = (seq1Ctr + 1) & 0x3;

   //Obviously the petrol consumption counter cannot handle
   //negative values. So we accumulate regen energy and
   //subtract it from the consumption once we're out of regen
   if (consumptionIncrement >= 0)
   {
      if (accumulatedRegen > (uint32_t)consumptionIncrement)
      {
         accumulatedRegen -= consumptionIncrement;
         consumptionIncrement = 0;
      }
      else if (accumulatedRegen > 0) //greater 0 but less than current draw
      {
         consumptionIncrement -= accumulatedRegen;
         accumulatedRegen = 0;
      }
      consumptionCounter += consumptionIncrement;
   }
   else
   {
      accumulatedRegen += -consumptionIncrement;
   }

   Param::SetFloat(Param::power, power);

   SimulateOilSensor();
   SetRevCounter();

   if (Param::GetInt(Param::invdir) == DIR_REVERSE)
   {
      DigIo::rev_out.Set();
   }
   else
   {
      DigIo::rev_out.Clear();
   }

   if (Param::GetInt(Param::invdir) == DIR_FORWARD)
   {
      DigIo::fwd_out.Set();
   }
   else
   {
      DigIo::fwd_out.Clear();
   }

   if (invmode == MOD_RUN)
   {
      if (vacuum > vacuumthresh)
      {
         DigIo::vacuum_out.Set();
      }
      else if (vacuum < vacuumhyst)
      {
         DigIo::vacuum_out.Clear();
      }
      Param::SetInt(Param::opmode, MOD_RUN);
   }
   else
   {
      DigIo::vacuum_out.Clear();
   }

   //float cur = 1000 * Param::GetFloat(Param::chglim) / udcbms;
   //cur *= Param::GetFloat(Param::powerslack);
   Param::SetInt(Param::vacuum, vacuum);
   //Param::SetFloat(Param::chgcurlim, cur);
   //cur = 1000 * Param::GetFloat(Param::dislim) / udcbms;
   //cur *= Param::GetFloat(Param::powerslack);
   //cur = MIN(511, cur);
   //Param::SetFloat(Param::discurlim, cur);

   //ReadDirectionButtons();
   GetDigInputs();
   ProcessThrottle();
   LimitThrottle();
   ProcessShifLever();
   ProcessParkButton();
   ProcessParkActuator();
   SendGearMessage();
   SendSpeedMessage();

   ErrorMessage::SetTime(rtc_get_counter_val());

   //LeafBMS::Send10msMessages(can, dcdcVoltage);
   SetFuelGauge();

   uint32_t canData[2];

   //Byte1 seq 2, Byte ?, Byte 7 XOR(bytes[0..6])
   //uint8_t check = seq2[seq1Ctr] ^ errlights ^ (consumptionCounter & 0xFF) ^ (consumptionCounter >> 8) ^ cruiselight ^ 0x1A;
   //canData[0] = seq2[seq1Ctr] | errlights << 8 | consumptionCounter << 16;
   //canData[1] = 0x1A | cruiselight << 18 | check << 24;

   //can->Send(0x480, canData);

   //if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      //can->SendAll();
   typedef struct {
    unsigned char SixBits:6;
    unsigned char TwoBits:2;
   } tEightBits;
   tEightBits canrun;
   tEightBits canio;
   canio.SixBits = Param::GetInt(Param::canio);
   canrun.TwoBits = g_canrun;
   canData[0] = 0 | canio.SixBits << 24 | canrun.TwoBits << 30;
   canData[1] = 0 | canrun.TwoBits << 14;
   can->Send(0x03F, canData);
   //canrun.TwoBits = 1;
   //canData[0] = 0 |canio.SixBits << 2 | canrun.TwoBits;
   //canData[1] = 0 | canrun.TwoBits << 12;
   //can->Send(0x03F, canData);
   //canrun.TwoBits = 2;
   //canData[0] = 0 | canio.SixBits << 2 | canrun.TwoBits;
   //canData[1] = 0 | canrun.TwoBits << 12;
   //can->Send(0x03F, canData);
   if (g_canrun < 2)
      g_canrun++;
   else
      g_canrun = 0;
   }

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   if (Param::canspeed == paramNum)
      can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));

   fuelGaugeController.SetMinMaxY(Param::GetInt(Param::fueldcmin), Param::GetInt(Param::fueldcmax));
}

static void CanCallback(uint32_t id, uint32_t data[2])
{
   switch (id)
   {
   case 0x030:
      ReadBrakePedal(data);
      break;
   case 0x108:
      //ChaDeMo::Process108Message(data);
      break;
   case 0x109:
      //ChaDeMo::Process109Message(data);
      break;
   case 0x420:
      //Param::SetFloat(Param::tmpaux, (((data[0] >> 8) & 0xFF) - 100) / 2.0f);
      break;
   case 0x0B4: //SKID_ECU VehicleSpeed_B4: byte5:6 signed, 0.009765625 km/h/count
   {
      uint8_t* bytes = (uint8_t*)data;
      int16_t raw = (int16_t)((bytes[5] << 8) | bytes[6]);
      g_vehicleSpeedKmh = raw * 0.009765625f;
      break;
   }
   default:
      LeafBMS::DecodeCAN(id, data, rtc_get_counter_val());
      break;
   }
}

static void ConfigureVariantIO()
{
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   AnaIn::Start();
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup();
   rtc_setup();
   write_bootloader_pininit();
   ConfigureVariantIO();
   tim_setup();
   nvic_setup();
   parm_load();

   LinBus l(USART1, 19200);
   Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));

   c.SetNodeId(2);
   c.SetReceiveCallback(CanCallback);
   //c.RegisterUserMessage(0x7BB);
   //c.RegisterUserMessage(0x1DB);
   c.RegisterUserMessage(0x1DC);
   c.RegisterUserMessage(0x55B);
   c.RegisterUserMessage(0x5BC);
   c.RegisterUserMessage(0x5C0);
   c.RegisterUserMessage(0x108);
   c.RegisterUserMessage(0x109);
   c.RegisterUserMessage(0x420);
   c.RegisterUserMessage(0x3CB);
   c.RegisterUserMessage(0x030);
   c.RegisterUserMessage(0x0B4);
   

   can = &c;
   lin = &l;

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   Terminal t(USART3, termCmds);

   fuelGaugeController.SetGains(1, 1);
   fuelGaugeController.SetCallingFrequency(100);
   fuelGaugeController.SetMinMaxY(Param::GetInt(Param::fueldcmin), Param::GetInt(Param::fueldcmax));

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms16Task, 16);
   s.AddTask(Ms100Task, 100);

   Param::SetInt(Param::version, 4); //COM protocol version 4
   Param::SetInt(Param::tmpaux, 87); //sends n/a value to Leaf BMS
   Param::SetInt(Param::compressor, 0); //Make sure we don't load this from flash
   Param::SetInt(Param::soc, 100); //Preload SoC for proper fuel gauge display

   while(1)
      t.Run();

   return 0;
}

