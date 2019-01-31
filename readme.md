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

- Command: A command is a single action that your robot can do. For example, you might have a command to intake a ball. This command would call a function from your intake subsytem that says run a motor at 50 percent speed. Commands can be triggered in multiple ways. The mose usual being through button presses. They can also be triggered manually, or through CommandGroups. Each command has 5 phases in its lifecycle
    1. Constructor - When you create a new instance of a command. Pretty standard. One thing to note is that if you are using the OI class to create commands, it will create a new instance for each button press. This can be pretty confusing.
    2. Initialize - This function runs only the first time this command is created. This seems counter-intuitive but thats just the way it is. So if you press a button more than once and the command is created more than once, Init would only be called once.
    3. Execute - This is where the main functionality of your command goes. This function will continously run until IsFinished returns true, or the command is interupted.
    4. IsFinished - This function is run right after the execute function each cycle. If it returns true, the command ends. If false, the command will run another execute cycle again. This can be useful for peacefully ending commands.
    5. End - This commands runs after IsFinished returns true. This can be used for clean up after a command is done.
    6. Interupted - This command is called if another command of the same subsystem gets a higher priority in the Scheduler queue. It will interrupt the command, and kill it. 

    A command can also Requires() a subsystem to make it belong to a subsystem.

- Command Groups: A command group is a sequence of commands that can run sequentually or in parallel. You have to be very careful with this because depending on the order of your commands you might end up running things in parallel when you meant to run them in order. For example, you could have a command group that runs a command called IntakeBall and then another command called DriveForward. Another thing to note is that commands are added to commandgroups in their constructor so you have to be careful with timing issues.

## Chapter Five: Tigertronics Specifics

Over the years of Tigertronics, I have developed a system to quickly put together robot code that works in 99% of use cases. A lot of the commands can be copy-pasted and will work without issue. Lets look at some code that I have created that allows us to use Commands in auto and teleop seamlessly.

For example lets look at a command function by function:

```C++
ControlIntakeWheels::ControlIntakeWheels(double time, double speed) {
  Requires(Robot::intakeSubsystem.get());
  timer = std::make_unique<frc::Timer>();
  timeCurrent = 0;
  isDone = false;
  inputSpeed = speed;
  timeTarget = time;
  timer->Reset();
  timer->Start();
}
```

We take in two parameters, a time and speed. We create a timer and set our parameters up. We also start the timer.

```C++
void ControlIntakeWheels::Execute() {
    if(inputSpeed == 0) {
		Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
		isDone = true;
    }
    else {
        timeCurrent = timer->Get();
        if(timeTarget == 0) {
            Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
            isDone = false;
        }
        else {
            if(timeCurrent >= timeTarget) {
                Robot::intakeSubsystem->SetIntakeWheelSpeed(0);
                isDone = true;
            }
            else {
                Robot::intakeSubsystem->SetIntakeWheelSpeed(inputSpeed);
                isDone = false;
            }
        }
    }
}
```
The time parameters are used to tell if we are in Autonomous Mode or Teleoperated. If the time is zero, we set our wheelspeed in the subsystem. and set isDone to false. This means the command will keep running until we let go of the button. Then, when the buttons is released, OI created a new command that has speed and time zero. This causes us to stop the wheels and the command ends immediatley. 

If the time is not zero, we look at the current time and check if it is past the time we want it to execute. If it isnt we continue setting the speed to our target, if it is, we end the command and set our wheel speed to zero.

This type of command layout works for a lot of the mechanisms on our robot. For example, intakes, or anything that moves freely without any more advanced control.

If we do want more advanced control like PID or motion profiling, the command actually becomes more simple (If you are using the talonSRX for the control). This is because you configure the talon at the start and just tell it to move to a certain point. For example, here is a snipped version of our elevator command. The PID and motion profiling runs on the Talon.

```C++
void ElevatorControl::Execute() {
	yAxis = Robot::oi->GetOperatorController()->GetLeftYAxis();
	isRightShoulderPressed = Robot::oi->GetOperatorController()->rightShoulderButton->Get();
	isLeftShoulderPressed = Robot::oi->GetOperatorController()->leftShoulderButton->Get();
        <snipped>
	if (manualControl) {
		Robot::elevatorSubsystem->RunElevatorMotor(yAxis);
	}
	else {
		switch(currentState)
		{
		case GROUND:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_GROUND);
			break;
		case LEVEL_ONE:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_ONE_HATCH);
			break;
		case LEVEL_TWO:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_TWO_HATCH);
			break;
		case LEVEL_THREE:
			Robot::elevatorSubsystem->GoToHeight(Robot::robotMap->kELEVATOR_LEVEL_THREE_HATCH);
			break;
		}
	}
}
```

In our subsystem those functions just call the Set Method on the talon with the specified setpoint in ticks.

## Chapter Six: Pose Estimation (May require some background calculus knowlege)

To do more advanced autonomous modes, we need a way of measuring where we are on the feild, to do this we need what we call Pose Estimation. This means that we can estimate the current "state" (meaning x, y, and yaw) of our robot using encoders. To do this we can use a set of equations called Forward Kinematics. The input to this equation is our wheel velocities, and the output is our total translational and rotational velocity vectors. This isn't useful by itself until we integrate those values to get position.

Here is the specifics of how the system works.

1. Every "tick" of the roborio (0.02 seconds), We get the distance that each wheel has traveled. Then add that to the old value of the distance which is the last iterations distance. This gives us a "delta" distance. We can then turn those changes in position into change in velocity by dividing by our delta time.

2. We then pass those values into our forward kinematics equation. This equation for mecanum at least was found on a research paper online. The details of that and the derivations are over my head but it involves a lot of linear algebra haha. This equation outputs the global x, y and yaw velocity vectors.

3. These global x, y and yaw vectors are then translated by the previous loops robot state. This gives us a position of the robot that is updated every loop.

A lot of the math and technqiues used are borrowed from Team 2481 and 1323. The "math" classes are basically directly copied. Also, we shouldn't expect this to be too accurate because of the slippage of the mecanum wheels. 

## Chapter Six Point Five: Path following

So the pose estimation helps us find where we are on the feild, but we want to control where are robot goes precisely in autonomous. A "Path" consists of a time, x, y, and yaw coordinates to go to. We then add these coordinates to a map which the robot iterates through at each time step and then tell the robot to move to that position. We use 3 PID loops on our x, y and yaw at the same time to move across the field. This works decently, but because of the way mecanum wheels strafe, you need to find a "smudge factor" because you won't move at full speed sideways as you would forward. 

To generate the path, is a work in progress, right now you can use pathweaver and then manually adjust the CSV file to the format you need.

FORMAT:
time, x, y, yaw

You then upload those CSV files to the roborio using WinSCP. You copy them to /home/lvuser/

## Chapter Seven: Tips and Tricks learned

### Talon SRX Tips
- To correctly tune PID loops and make sure everything is working correctly, make sure your hardware is set up correctly. A common thing to do is to hook up the white wire off of the talon to the red wire on the motor and the green wire to the black wire. Then make sure the encoder is securely mounted to the shaft of the motor or output shaft (etc). Then make sure you have wired it directly to the talon through a breakout board or if it is CTRE Mag encoder, ensure it is clear of debris when inserting it into the slot on the talon. Then, load up a very basic program that lets you control the motor with a joystick and make sure when you set the talon to a positive direction, its lights blink green and the encoder position increases. THIS HAS TO BE SET CORRECTLY OR ELSE PID WILL NOT WORK. If it is incorrect, you can invert the output of the motor or reverse the sensor. Now, make sure you also multiply the output by the gear ratio or a ticks per revolution of the encoder. This is so you can apply real world measurements and make it easier to make sense if something is going wrong. 

- Sometimes, Talons will show up in the configuration tool, this could be because it needs new firmware, or more likley, it has a duplicate ID as another talon. So re-ID one and see if that works

### PID Tuning Tips

- Start with P at 1, I and D at 0. Then run a program that moves a mechanism to a position and observe. If the mechanism doesnt reach its setpoint, double P. If it spins or moves wildly, half P. The ideal P value is one where the mecahnism occilates around its setpoint without spinning out of control. Then, you can increase D until the mecanism stops occilating around the setpoint. If it still doesnt reach its setpoint, increase I until it does. Start with D at 10 if P is 1 and I at 0.01;

- If nothing seems to be working, make sure you have your sensors and exepected values set up correctly. You might have something reversed and not realize it.

### Networking

- Always use static IP addresses for anything connected to the robot. The roborio and radio do support mDNS, but it is not very fast and could lead to problems down the road. 

- RoboRIO ip is always 10.20.53.2 if connected over wifi, if connected over USB, you can connect to the roboRIO using 172.22.11.2

### RoboRIO

- If the roborio isnt connecting to the radio, but your computer is connecting to the radio, make sure the roborio is actually on and the ethernet cables are connected. If that doesnt work, try and reflash the image of the roborio.

- If your driverstaion keeps saying "No Robot Code" you either have no frcUserProgram or your code crashes instantly. This usually means some sort of nullptr error or memory error. Add some print statments or debug.

- Pay attention to the status LEDS on the roborio it might tell you something important.

### Radio

- To configure the radio, you must use the FRC Radio Configuration Tool. You must re configure the robot after compeitition because it is a different config.

- It takes a bit to connect sometimes, be patient.

### NavX

- If the navX is giving very strange values for yaw, it might be because it was calibrated in a different orientation from before. When calibrating, it must be inline with earths axis. So, it must be vertical or horizontal and NOT at an angle.

- Even though the NavX is powered through the mxp port on the roborio, it is a good idea to use a USB cable to power it through the roborio as well. This is because the voltage will cut out the the mxp ports first rather than the usb ports if the battery voltage gets too low.

### Raspberry PI Vision Stuff

- Using the FRCVision tool, you can easily upload cross compiled rpi programs to the roborio and it will hangle restarting, power cutoff etc. There is a readme in the repo for the vision that explains how to set it up.

- To increase the update rate of the network tables, you can set the update rate to a very slow value, then flush the values every iteration of the loop.

### CAN Stuff 

- If the motors are not spinning, or showing up in the configuration tool, make sure your CAN wires are connected correctly, yellow to yellow and green to green. Also make sure the Talons are actually on. 
