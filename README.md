# Play Bot Code

This is the code for our 2026 Play Bot in collaboration with the theatre people at HVA.

## What do I need to install?

There are a few things that you need to develop and run code. You can use the guide provided by FRC, [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html). OR you can download everything you need in these two links: [Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#500107), and [WPIlib dev tools](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2). Quick note: you will need the WPIlib VS code extension installed. If it doesn't work or isn't there, this whole thing won't run.

As for dependencies: You will need to download [Java](https://javadl.oracle.com/webapps/download/AutoDL?BundleId=249833_43d62d619be4e416215729597d70b8ac) and you will probably need (visual studio)[with Desktop cpp development)](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&channel=Release&version=VS2022&source=VSLandingPage&cid=2030&passive=false) but you will only really need it for simulation though. The rest of the things you need will be installed by the WPI extension on VScode.

## How to connect code to hardware

Hardware is fairly simple. You find the documentation, and you implement those methods. The hard part is finding what you need and deciphering how they want you to implement it.

* [CTRE Phoenix 6](https://api.ctr-electronics.com/phoenix6/release/cpp/)
* [REVlib](https://codedocs.revrobotics.com/cpp/classrev_1_1_spark_max_p_i_d_controller)

These are similarly laid out, which makes finding things simple, and seeing which one you're on confusing. The main things we need are called "classes" and "namespace". The class list is where you can find all you need to use the motors. We use namespaces when you don't want to type out "rev::hardware::sparkmax::etc::etc" when you want to use that method. You can use it as an implied shortcut. If you have something like "ctre::phoenix6::hardware::TalonFX()", then you can use "using namespace ctre::pheonix6::hardware;" to shorten the definition to just "TalonFX()". CPP can assume that you want a class from that library.

## Motor Control

"Control systems are all around us and we interact with them daily. A small list of ones you may have seen includes heaters and air conditioners with thermostats, cruise control, and the anti-lock braking system (ABS) on automobiles, and fan speed modulation on modern laptops. Control systems monitor or control the behavior of systems like these and may consist of humans controlling them directly (manual control), or of only machines (automatic control)." - WPI Docs/Tyler Veness

[Also, see the supplementary WPI docs article.](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/control-system-basics.html)

### P.I.D.

[PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html) or Proportional–integral–derivative, is how we accurately and smoothly move motors. Its a "feedback" controller, meaning that it uses some sensor data to reach a setpoint. Often, this means inputting encoder position and a desired setpoint into a calculator which, then, yields the output for the motor. P, I, and D are "constants," numbers that we choose that define how we calculate the motor output.

P - With P we multiply it with the "error" and add that to the motor output. The "error" is the current position subtracted from the setpoint or, in other terms, it's the difference between where we are and where we want to be.

I - I is the integral, which "averages" all of the past output values towards the average of all past setpoint values. Over time, with more error, the integral term adds more and more to the output, and, with less and less error, the value added goes down. This is generally used to combat a "static" or unchanging force working against the force of the motor (ie gravity). Generally, in FRC, its advised against because an "open-loop" controller would be better; [see here for more information](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#integral-term).

D - The use of the "derivative" term which adds to the output proportionally to the "derivative" of the error. The derivative is like the instantaneous rate of change; imagine the slope of a curve calculated in one single spot. In practice, derivative action predicts system behavior and thus improves settling time and stability of the system (Wikipedia). This helps with dampening "overshoots," going beyond the setpoint, and "undershooting," going under the setpoint. Generally, using dampening like this is significantly more stable than using a pure P controller, as the controller becomes "softer."

### S.V.A.

[SVA](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html)

Feedforward controls, often called S.V.A. (Static, Velocity, Acceleration) or sometimes just kV/kA, are "open-loop" controllers that predict what motor output we'll need before we get there. Unlike PID which reacts to error after it happens, feedforward proactively calculates the voltage needed based on physics and our desired motion profile. Open-loop control also does not require any encoder input.

S (Static) - This constant overcomes static friction and other forces that resist initial movement. It's the minimum voltage needed just to get the mechanism moving from a standstill. Think of it like the push needed to overcome stiction in gears or the voltage to hold an arm against gravity. This is often the main replacement for the I PID term.

V (Velocity) - The velocity constant relates desired velocity to required voltage. We multiply kV by our target velocity to get the voltage needed to maintain that speed. For example, if we want a flywheel spinning at 100 rad/s, and kV = 0.5, we'd apply 50 volts. This is usually the most important feedforward term.

A (Acceleration) - This constant accounts for the voltage needed to accelerate or decelerate the mechanism. We multiply kA by our target acceleration. This helps when speeding up or slowing down, making motion profiles smoother and more accurate. It's especially useful for preventing lag when starting movement or overshooting when stopping.

In practice,if used, feedforward is almost always paired with PID. Feedforward gets you close to the target quickly, while PID corrects any remaining error. This combination is much more responsive than PID alone, since you're not waiting for error to build up before the controller reacts.

## Joysticks

Joysticks use the standard FRC library. Some great documentation and a tutorial can be found [here](https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html). You can use almost any controller including Dualshock controllers, Xbox controllers, and joysticks. I have also seen custom driver stations that use homemade controllers. We made one of these a few years back and should still have it. However, I don't know if there's any reuseable code.

## Swerve

Swerve is the preferred drive train for our team, and we've used it for multiple years. The math behind Swerve code is deceptively difficult, it requires lots and lots of trigonometry. That is why we use papers written by people who have spent many more hours than they'd probably like to admit. Particularly, you can find almost everything that you need in these two Chef Delphi threads:

* [LOTS of papers](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383)
* [3 papers that are extremely useful](https://www.chiefdelphi.com/t/math-and-programming-behind-swerve/130241/9_)
thx Ether

If you want the base swerve code you would copy and paste the paper on the second link labeled "How to use the equations:"; which we've already done. However, these aren't exactly perfect. The turning while moving works, but not that well; you can only go in one direction; and the wheels always go back to zero. These are all very fixable problems that have been solved before, and that we will work on.

## Contributing

To contribute, you need to be part of the HVA Robotics organization. Then, you can create a branch, work on that, then merge once everything is ready. However, you can commit the tested code directly to the main branch when on the drive laptop. 

Writing and testing code can take some time to set up, but after that, it's a matter of writing good code (as opposed to fighting the compiler). You need the dev tools that can be installed (if you haven't already): [WPIlib dev tools](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2).

A good way of contributing is updating all the libraries to the newest. You can do that by running the code and trying to clear up the page and a half of "warning!" messages. As 
a quick note: It will still build with these warnings.

## Testing

Testing is the fun part. The process is as follows: Connect the laptop to the robot via a cable (USB-A or Ethernet directly into the RoboRIO), or through the radio; then open up the driver station and verify that you have robot communication and joysticks; finally, you can deploy the code.
Some of these aren't very simple so here are instructions for some of them:
**Deploying Code**
You should've already installed VS code in the "What do I need to install step", but if you haven't you can do that [here](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2).
1. When in VS code, press ctrl+p or click the WPIlib logo in the top right.
2. Search for "Build code" then, select that.
3. That should've opened up a terminal that compiles that code, which you can debug from.
4. Make sure you are connected to the robot in the Driver Station.
5. Go to the WPIlib menu (ctrl+p or click the logo) and "select deploy code".
6. Once it compiles, it pushes the code to the RoboRIO. After that, all the init functions should've been called and the controllers should now work.

**Connecting to the Radio**
1. Connect the roboRIO directly into the radio with an Ethernet cable (make sure that it's plugged into the middle port)
2. Open up the radio utility put in your team number and set "Radio:" to "OpenMesh".
3. Connect to the robot through WIFI.
4. Deploy from WPI VS code extension.
NOTE: make sure to update firmware from time to time

## Troubleshooting

When a silly build member walks up to you and says the code is broken, remain calm. If the code has been tested before, and it works, pretend to do something that looks like code, and have them also try troubleshooting on their end. Half the time it's a build problem, half the time you wrote bad code or bad math. If it's not a problem on either end, it's likely a problem with the hardware/firmware. Don't be afraid to ask for help if you suspect it's a programming problem, no one knows how to do this when they have just started

on a more serious note:
**Chef Delphi**

Chef Delphi is *the* forum for FRC programming. Most problems that aren't already in stack overflow can be found here. It's a forum like any other that people in robotics use. It's essential for troubleshooting and generally has a lot of general guides. However, a lot of it is in Java, so you might have to try and interpret Java code when working in C++.

### Note:
Most of this was written when I was sick, there are definitely a lot of mistakes, typos, etc. Please send pull requests to fix!