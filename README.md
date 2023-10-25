# stm32-car
~~This firmware is a hacky mishmash of VW and Nissan CAN code. It talks to the Nissan BMS (aka LBC) to find out cell voltages and such. It also produces all messages needed for satisfying the various controllers in newer VW cars (my testbed is a 2004 Touran). That means all warning lights are off.~~
~~It also implements the ChaDeMo protocol and has been successfully tested on various fast chargers.~~

# Features
~~- Send all messages necessary to make the DSC light go off~~
~~- Do simple traction control by comparing rear and front axle speeds~~
~~- Fake the oil pressure sensor depending on motor rpm~~
~~- Map the electric motor speed onto the speed dial~~
~~- Map the electric motor temperature onto the temp dial~~
~~- Map the momentary electric power onto the fuel economy display~~
~~- Read key switch, brake switch and cruise control buttons from CAN bus~~
~~- Read brake vacuum sensor and control vacuum pump~~
~~- Read throttle pedal and put it on the CAN bus~~
~~  - Including support for dual channel throttles~~
~~- Read individual cell voltages from Nissan Leaf BMS (LBC)~~
~~- Read SoC, SoH etc. from LBC~~
~~- Read power limits from LBC and use them for inverter power limit and charge current control~~
~~- Implement ChaDeMo protocol with values obtained from LBC~~
~~- Control analog fuel gauge via two current source channels~~
- Read the Prius shift leaver and set the direction for the inverter

# IO pins
~~- throttle1 (pot) - PC1~~
~~- throttle2 (pot2) - PC0~~
~~- Vacuum sensor - PC2~~
~~- 12V level - PA3~~
~~- digital start input - PB6~~
~~- digital brake input - PA2~~
~~- DC switch output - PC13~~
~~- Precharge output - PC11~~
~~- Vacuum pump output - PB1~~
~~- Oil sensor simulation - PC5~~
~~- Fuel sensor simulation - PB7, PB8 (positive, negative)~~

# CAN configuration (VW)
Not all CAN messages are hard coded, some are configured via the generic interface.
Send these commands:

~~- can tx speedmod 640 16 16 4 //speed dial with simulated idling engine so that power steering works~~
~~- can tx tmpmod 648 8 8 1 //temperature dial~~
~~- can tx pot 1 0 12 1 //this is for the inverter, configure however you want~~
~~- can tx canio 1 12 6 1 //this is for the inverter, configure however you want~~
~~- can tx cruisespeed 1 18 14 1 //this is for the inverter, configure however you want~~
~~- can tx potbrake 1 32 9 1 //this is for the inverter, configure however you want~~
~~- can tx heatcur 1 41 9 1 //this is for the inverter, configure however you want~~
~~- can tx calcthrotmin 1 50 7 -1 //this is for the inverter, configure however you want~~
~~- can tx calcthrotmax 1 57 7 1 //this is for the inverter, configure however you want~~
~~- can rx din_start 1394 3 1 32 //grabs key switch start position~~
~~- can rx espoff 416 9 1 32 //grabs ESP off button~~
- can rx udcinv 3 16 16 8 //various inverter values
- can rx tmpm 3 48 8 32
- can rx speed 3 32 16 32
- can rx uaux 3 8 8 4
- can rx opmode 3 0 3 32
~~- can rx cruisestt 906 8 4 32 //grabs cruise control buttons~~
~~- can rx wheelfl 1184 4 12 5 //Wheel speeds~~
~~- can rx wheelfr 1184 20 12 5~~
~~- can rx wheelrl 1184 36 12 5~~
~~- can rx wheelrr 1184 52 12 5~~
~~- can rx handbrk 800 9 1 32 //Hand brake pulled?~~
~~- can rx brakepressure 1192 16 15 32 //Brake pressure for added regen~~

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedencies are libopencm3 and libopeninv. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-car by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
