# Bond Brigade Reefscape 2025 Code

## Subsystems:
- Swerve<br>
&nbsp;&nbsp;&nbsp;YAGSL code generated for S.D.S. MK4 swerve modules with vortexes instead of Neo 550s.

## Setup Instructions
1. Clone this Repository.
2. Ensure that WPILIB is updated to a ***2025*** version.
3. Update the firmware of your RoboRio to a ***2025*** version.
4. Update the firmware of your SparkMax's and CANCoders to the newest version.
5. Configure the ID's for your drive motors, turn motors, and CANCoders.
6. **OPTIONAL:** Install FRC WEB Components - This will make troubleshooting the offsets of the modules **much** easier.
    - Download the repo from [here](https://github.com/frc-web-components/frc-web-components)
    - Once the app is installed and opened, go to settings and change the I.P. address to 10.TE.AM.2 (10.55.86.2 for our team).
    - Then go back to the editor and select the components menu. Then drag out ```Swerve Drivebase```, and you should see the output data from your drive base.
7. Configure the offsets for your angle motors.
8. Configure the PID values and speed for your motors.


## Project Structure

- ```deploy/swerve``` YAGSL Config
    - ```swervedrive.json``` Configures the Gyro and module positions.
    - ```modules/pidfproperties.json``` Configures the PID properties for all the drive and angle motors.
    - ```modules/physicalproperties.json``` Configures the physical properties of the modules, like gear ratios, also controls the current the motors can recieve.
    - ```modules/module_name.json``` Configures the CAN ID's for the motors and encoders. Also declares offsets for the module.
-  ```frc/robot``` Contains the main code
    - ```subsytems``` Contains the code for the subsystems like swerve.
    - ```Constants.java``` Stores constants the subsystems, including configs not included in YAGSL JSON files.
    - ```RobotContainer.java``` Initializes the subsystems, and configures controls.


## Controls:
- Driver controls:
    - Left Joystick X-axis: X-axis movement
    - Left Joystick Y-axis: Y-axis movement
    - Right Joystick X-axis: Rotation
