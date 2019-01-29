# Writing FRC Robot Code for 2019 Deep Space

## Chapter One: Background

### __First off, I will assume the reader has a basic understanding of programming including Object Oriented Programming, different data types and other introductory subjects.__

FRC Robots can be programmed in many different languages, however only three are officially supported.

- Java
- C++
- Labview

However, there are more options if you are willing to give up support from event staff. Some include:

- [Python](https://robotpy.readthedocs.io/en/stable/) (the best supported non official language)
- Scala (Any JVM language will work.)
- Kotlin
- C# (Rare)
- Rust (Barely working)

I would recommend sticking with a language that your programming mentor knows best or one that is the three offical languages.

The computer that is the brain of the robot is called the RoboRIO. It is a [Linux](https://en.wikipedia.org/wiki/Linux) computer with and [FPGA](https://en.wikipedia.org/wiki/Field-programmable_gate_array) attached. This FPGA allows the computer to control and read from different hardware that is attached to robot such as limit switchs, sensors, and motor controllers. 

## Chapter Two: Setting up the equipment

I am going to link to [WPI Screensteps](https://wpilib.screenstepslive.com/s/4485) Live because it is an excellent resource to setup FRC Programming Software.

Follow the directions under "Getting Started with the Control System" and real the articles relevant to your programming language before continuing. 

When working on FRC Robots, you need to make sure all of your hardware is up to date. Usually, every year, National Instruments (who makes the RoboRIO) releases a new firmware version or image version. This is **VERY** important because when going to competition, you require certain firmware versions. This is unlikley to change throughout build season but it may, so watch out for Team Updates from FIRST. 

You also need to make sure all 3rd party dependencies are updated as well. This included equipment like CTRE Talon SRX's, REV Robotics Spark Max, and the Kauai Labs NavX. They all have firmware that needs to be updated every year and especially for newer products, have multiple updates throughout the build season that may fix bugs. 

To install these libraries and use them in our code, we can simply add their libraries build repo using GradeRIO. To do this do Ctrl+Shift+P in VSCode in a robot project, and search for Manage Vendor Libraries, and follow the directions from the vendors site. You can also check for updates using the same method. 

If you run into a wierd issue, don't be afraid to search on Chief Delphi or email the support team for the company. They are always willing to help.

## Chapter Two and a Half (Advanced): Build Systems
To build the code we use the [Gradle](https://gradle.org/) build system. To make this easier, the developers who maintain all the libraries that help you code the robot made a plugin for Gradle called [GradleRIO](https://github.com/wpilibsuite/GradleRIO). This makes it extremely simple to build and deploy code on any OS or platform. You can use and IDE as long as it supports Gradle. You could even use a text editor and build from the terminal. This works on windows, mac and linux. You might have trouble finding a roborio toolchain on obscure distros though. (I found one for Arch so maybe not).

When you build code, you are actually cross compiling code for the roborio because it is an ARM processor. This program is called frcUserProgram. When you deploy, you are killing off the previously running frcUserProgram and copying over the program on your computer to the RoboRIO.


## Chapter Three: Terminology

- RoboRIO - The brain of the robot, runs linux and this is what you write programs for
- Motor Controller - Reads signals from the RoboRIO over CAN or PWM and sends voltage to a motor to move at a specific speed. Some have more advanced features like built in position control, velocity control and even motion profiling. Ex: TalonSRX, Spark Max, Victor SPX
- IMU - Inertial Measurement Unit - This is a sensor that can report back the acceleration of the sensor, as well as tilt, roll and yaw of the sensor. We usually use this to detect that angle that the robot is at relative to the field.
- HAL - Hardware Abstraction Layer - This is code that communicates between the FPGA and the processor. This allows you the programmer, to easily control the different I/O of the robot without having to worry about threading / saftey and stuff like that.

## Chapter Four: Framework
 I would highly recommend looking over [this](https://cpb-us-w2.wpmucdn.com/wp.wpi.edu/dist/1/35/files/2018/05/Building-and-Contributing-to-WPILib-2018.pdf) Powerpoint created by the people at WPI to understand how the build system and framework works.
Here is a general overview of how our code interacts with the lower layers of software. ![RoboRIO Software](https://i.imgur.com/LsR4ZPu.png) Our code lives above the blue box on the left. We write code that interacts with WPILib C++. 

There is a lot to this section so lets jump in! I will use C++ as an example here because that is what I know best.

So to start, your main class is going to be called Robot. Its files are stored in /src and /include when you create a project through VSCode. This class inherits from a RobotBase class provided by WPI. There are different types of classes that you can inherit from. One of them is TimedRobot which is the most common and what you want to use 99% of the time. There is also, an iterative robot, as well as sample robot. There is also what we use which is called __Command Based Robot__. This is based on TimedRobot, but is a differently structured to allow programmers to do more than one action at once (sort of). 

### Command Based Architecture

The Command Based system is very powerful but definetly has its quirks. The whole system is run off of a [Queue](https://en.wikipedia.org/wiki/Queue_(abstract_data_type)) of Commands or Functions. This is called the Scheduler. You mostly dont have to worry about the Scheduler unless you are doing advanced things with Autonomous or switching commands.There are a couple different terms you need to be familiar with. 

- Subsystems: A subsystem can be considered a system of your robot. For example you might have a intake mechanism on your robot as well as a drivetrain, and a shooter. All of these could be considered different subsystems as they act independantly of each other. Each subsystem can have a single running "Command" that is using that subsystem. Each subsystem can also have a default command that can run when no other commands are running.

- Command: A command is a single action that your robot can do. For example, you might have a command to intake a ball. This command would call a function from your intake subsytem that says run a motor at 50 percent speed. Commands can be triggered in multiple ways. The mose usual being through button presses. They can also be triggered manually, or through CommandGroups.

- Command Groups: A command group is a sequence of commands that can run sequentually or in parallel. You have to be very careful with this because depending on the order of your commands you might end up running things in parallel when you meant to run them in order. For example, you could have a command group that runs a command called IntakeBall and then another command called DriveForward. Another thing to note is that commands are added to commandgroups in their constructor so you have to be careful with timing issues.



