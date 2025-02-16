# 1-DOF copter portable demonstrator (STM32L432KC)
An STM32 HAL example of PID control for a 1-DOF copter. The demo features an ESC (electronic speed controller) that supports DShot (Digital Shot) protocol, and an AHRS from Bosch Sensortec.

![1-DOF copter on tarmac](/Assets/Images/one_dof_copter_on_tarmac.jpg)

Video: [1-DOF copter in action](http://ufnalski.edu.pl/stm32_projects/one_dof_copter_bno055/one_dof_copter_in_action.mp4)

# Motivation
When you have an [ESC](https://github.com/ufnalski/dshot_pwm_dma_l432kc) and an [AHRS](https://github.com/ufnalski/ahrs_bno055_g474re) up your sleeve you should definitely combine them into a 1-DOF copter! Such a copter poses an excellent plant[^1] to elevate your control system design skills. Let's then play with practical PID controllers. Integrator anti-windup and band-limited differentiator will be your best companions for that journey.

[^1]: https://en.wikipedia.org/wiki/Plant_(control_theory)]

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32L4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Exemplary hardware
* Talon32 Slim 40A AM32[^2] 128K 3-6S (Skystars)
* V2306/V2207 V3.0 VELOX 2550KV (T-MOTOR)
* HQProp S4 Lemon Lime (Ethix)
* 2200mAh 11.1V 30C 3S1P LiPo battery (Gens ace)

[^2]: They used to be shipped with the BLHeli_32 firmware. Recently the manufacturer switched to the AM32 firmware. Both of them support DShot300 and can be used interchangeably in this mini-project if DShot300 is selected. Similar exchangeability is not possible if DShot150 is opt for - the AM32 firmware does not support DShot150, which is totally understandable within the context of modern drones. 

# Compatible software (configuration tool)
* [AM32-MultiRotor-ESC-firmware](https://github.com/am32-firmware/AM32) (AlkaMotors)

# New to practical PID controllers?
* [What Is PID Control? | Understanding PID Control, Part 1](https://www.youtube.com/watch?v=wkfEZmsQqiA) (Brian Douglas, MathWorks)
* [Anti-windup for PID control | Understanding PID Control, Part 2](https://www.youtube.com/watch?v=NVLXCwc8HzM) (Brian Douglas, MathWorks)
* [Noise Filtering in PID Control | Understanding PID Control, Part 3](https://www.youtube.com/watch?v=7dUVdrs1e18) (Brian Douglas, MathWorks)
* [A PID Tuning Guide | Understanding PID Control, Part 4)](https://www.youtube.com/watch?v=sFOEsA0Irjs) (Brian Douglas, MathWorks)
* [Important PID Concepts | Understanding PID Control, Part 7](https://www.youtube.com/watch?v=tbgV6caAVcs) (Brian Douglas, MathWorks)
* [PID Controller Implementation in Software - Phil's Lab #6](https://www.youtube.com/watch?v=zOByx3Izf5U) (Phil’s Lab)
* [Bilinear transform](https://en.wikipedia.org/wiki/Bilinear_transform) (Wikipedia)
* [Proportional-integral-derivative controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) (Wikipedia)
* [Integral windup](https://en.wikipedia.org/wiki/Integral_windup) (Wikipedia)
* [The Map of Control Theory](https://engineeringmedia.com/map-of-control) (Brian Douglas, Engineering Media)

> [!NOTE]
> My favorite anti-windup technic is the one discussed by Brian Douglas. You can switch between the two by commenting or uncommenting #define PHIL_S_LAB.

# New to the AM32 project?
* [AM32 MultiRotor ESC Firmware](https://am32.ca/)
* [AlkaMotors electronics inc.](https://github.com/am32-firmware)
* [AM32 ESC Unlocker](https://github.com/am32-firmware/AM32-unlocker)
* [Creating a PC Link to AM32 using an Arduino](https://github.com/am32-firmware/am32-wiki/blob/main/docs/guides/Arduino-PC-Link.md)
* [AM32's NEW configurator settings 100% Explained](https://www.youtube.com/watch?v=VCIpMOESqmw) (Chris Rosser)
* [Tuning AM32 ESCs for Ultimate Performance](https://www.youtube.com/watch?v=3SHzyUaypFw) (Chris Rosser)

![AM32 ESC Setup 1](/Assets/Images/am32_config_tool_1.PNG)

![AM32 ESC Setup 2](/Assets/Images/am32_config_tool_2.PNG)

> [!WARNING]
> Propellers that spin at tens of thousands rpm are not toys :exclamation: Safety first: [LEGO 43038](https://www.lego.com/en-au/pick-and-build/pick-a-brick?query=43038).

> [!WARNING]
> LiPo batteries are not toys either - learn to handle them before you use one in your project :exclamation:

# Sources and inspirations
* [BNO055 STM32 library](https://github.com/ivyknob/bno055_stm32) (Ivy Knob) (MIT license)
* [PID controller](https://github.com/pms67/PID) (Philip Salmony) (MIT license)
* [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) (Aleksander Alekseev) (MIT license)

# Call to action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2025_high_school/Control_Engineering_for_Hobbyists_2025_02.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.phils-lab.net/), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Feedback control systems - do try them at home :exclamation:

200+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned :exclamation:
