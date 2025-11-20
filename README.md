# Play Bot Code

This is the code for our 2026 Play Bot in collaboration with the theatre people at HVA.

## What do I need to install?

There are a few things that you need to develop and run code. You can use the guide provided by FRC, [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html). OR you can download everything you need in these two links: [Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#500107), and [WPIlib dev tools](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2). Quick note: you will need the WPIlib VS code extension installed. If it doesn't work or isn't there, this whole thing won't run.

As for dependencies: You will need to download [Java](https://javadl.oracle.com/webapps/download/AutoDL?BundleId=249833_43d62d619be4e416215729597d70b8ac) and you will probably need [visual studio with Desktop C++ development](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&channel=Release&version=VS2022&source=VSLandingPage&cid=2030&passive=false) but you will only really need it for simulation though. The rest of the things you need will be installed by the WPI extension on VScode.

## What do I need to know?
[Go through this website; it's fantastic.](https://docs.wpilib.org/en/stable/index.html)
[These are some papers on swerve. Very interesting read, but not necessary.](https://www.chiefdelphi.com/t/math-and-programming-behind-swerve/130241)
[CTRE Docs](https://v6.docs.ctr-electronics.com/en/stable/)
[REV Docs](https://docs.revrobotics.com/docs/ion)

## Contributing

To contribute, you need to be part of the HVA Robotics Github org. Then, you can create a fork, work on that, then merge once everything is ready. However, you can commit the tested code directly to the main when on the drive laptop.

Writing and testing code can take some time to set up, but after that, it's a matter of writing good code (as opposed to fighting the compiler). You need the dev tools that can be installed (if you haven't already): [WPIlib dev tools](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.3.2).

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

When a silly build member walks up to you and says the code is broken, remain calm. If the code has been tested before, and it works, pretend to do something that looks like code, and have them also try troubleshooting on their end. Half the time it's a build problem, and the other half of the time, it's a programming problem. If it's not a problem on either end, it's likely REV or the spirit of Terry A. Davis. Don't be afraid to ask for help if you suspect it's a programming problem.

on a more serious note:
**Chef Delphi**

Chef Delphi is *the* forum for FRC programming. Most problems that aren't already in stack overflow can be found here. It's a forum like any other that people in robotics use. It's essential for troubleshooting and generally has a lot of general guides. However, a lot of it is in Java, so you might have to try and interpret Java code when working in C++.

