// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double controllerDeadband = 0.3;
  }

	public static class AutoConstants {
		public static final double autoXController = 0.001;
		public static final double autoYController = 0.001;
		public static final double autoThetaController = 0;
		public static final TrapezoidProfile.Constraints autoThetaControllerConstraints = new TrapezoidProfile.Constraints(
				4 * Math.PI / 10,
				Math.PI / 4
		);
	}

  public static class SwerveConstants {
    //                      Max speed of the robot in feet
    public static final double maxSpeed = 4.5;
    public static boolean isFieldOriented = false;
  }

	public static class AlgaeConstants {
		public static final int motorId = -1, motorTwoId = -1;
		public static final double p = 0.002, i = 0.0, d = 0.0;
		public static final double maxSpeed = 0;
		public static final boolean motorInverted = true, motorTwoInverted = false;
	}

	public static class CoralConstants {
		public static final int motorId = -1, motorTwoId = -1;
		public static final double driveMotorMaxSpeed = 0, angleMotorMaxSpeed = 0;
		public static final double wheelDriveP = 0.002, wheelDriveI = 0.0, wheelDriveD = 0.0;
		public static final double angleMotorP = 0.002, angleMotorI = 0.0, angleMotorD = 0.0;
		public static final boolean wheelDriveInverted = false, angleMotorInverted = false;
	}

	public static class ElevatorConstants {

		public static final double elevatorMaxSpeed = 0;
		public static final double trolleyMaxSpeed = 0;

		public static final int elevatorMotorId = -1, elevatorMotorTwoId = -1;
		public static final double elevatorMotorP = 0.002, elevatorMotorI = 0.0, elevatorMotorD = 0.0;
		public static final double elevatorMotorTwoP = 0.002, elevatorMotorTwoI = 0.0, elevatorMotorTwoD = 0.0;
		public static final boolean elevatorMotorInverted = false, elevatorMotorTwoInverted = false;

		int trolleyMotorId = -1;
		public static final double trolleyMotorP = 0.002, trolleyMotorI = 0.0, trolleyMotorD = 0.0;
		public static final boolean trolleyMotorInverted = false;
	}
}
