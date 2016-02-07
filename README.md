# Obstacle-Avoiding-Robot---RoboCORE

This is the source code of a program for the obstacle avoiding robot based on [RoboCORE](https://robocore.io/) controller.
It was a group project but the entire software was written by me.

Code has to be used with RoboCORE SDK.

![](https://github.com/loruro/Obstacle-Avoiding-Robot---RoboCORE/blob/master/img/robot.jpg)

Task of the robot was to avoid obstacles by using ultrasonic sensor.
It consist of:
* [RoboCORE](https://robocore.io/) (without Intel Edison) controller.
* [Lego Ultrasonic Sensor](http://shop.lego.com/en-PL/Ultrasonic-Sensor-9846) which outputs distance to objects.
* Three [Lego Servo Motors](http://shop.lego.com/en-PL/EV3-Large-Servo-Motor-45502). Two of them for powering wheels and one for rotating ultrasonic sensor.
* Two [Lego Touch Sensor](http://shop.lego.com/en-PL/Touch-Sensor-9843) connected together with bumper. It's safe mechanism in the case where ultrasonic sensor doesn't detect obstacle and robot hits it.
* 6 AA batteries.
* Custom 3D printed part with metal ball creating ball transfer for supporting rear end of the robot.
* Couple of general lego parts.

Robot moves forward and constantly scans surroundings in front of him. If it detects obstacle directly in front of him, robot decides in which direction it should rotate, rotates and continue moving forward.
If there is no satisfactory way or it hits something with bumper, robot moves backward for a few seconds and scan again.

After turning on the robot it doesn't know position of ultrasonic sensor. Calibration process involves turning sensor counterclockwise until it detects very close obstacle, which is Lego brick placed on robot for this purpose. After detection it knows more or less position of sensor.

When the robot is connected by USB cable with device with Android it can be connected to [RoboCORE Platform](https://wiki.robocore.io/cloud:start) and remotely controlled.
