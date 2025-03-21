// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
	  public static final int kOperatorControllerPort = 1;
	  public static final double controllerDeadband = 0.3;
	}

	public static class AutoConstants {
		public static final double autoXController = 0.001;
		public static final double autoYController = 0.001;
		public static final double autoThetaControllerVal = 0.001;
		public static ProfiledPIDController autoThetaController = new ProfiledPIDController(
			AutoConstants.autoThetaControllerVal,
			0.0,
			0.0,
			AutoConstants.autoThetaControllerConstraints
		);
		public static final TrapezoidProfile.Constraints autoThetaControllerConstraints = new TrapezoidProfile.Constraints(
				4 * Math.PI / 10,
				Math.PI / 4
		);
		//Measured in feet per second and feet per second squared respectfully.
		public static final double maxVelocity = 3, maxAcceleration = 2;
	}

	public static class AlgaeConstants {
		public static final int motorId = 35, motorTwoId = 36;
		public static final double p = 0.001, i = 0.0, d = 0.0;
		public static final double maxSpeed = 0;
		public static final boolean motorInverted = true, motorTwoInverted = false;
	}

	public static class CoralConstants {
		public static final int motorId = 37, pivotMotorId = 38;
		public static final double wheelDriveP = 0.002, wheelDriveI = 0.0, wheelDriveD = 0.0;
		public static final double angleMotorP = 0.002, angleMotorI = 0.0, angleMotorD = 0.0;
		public static final boolean wheelDriveInverted = false, angleMotorInverted = true;
		public static final double maxPos = 322, minPos = 183.5;
		public static final double levelFourPos = 0, LevelThreePos = 295, levelTwoPos = 286;
		public static final double homePos = maxPos, sourcePos = 210;
		//L1: 242
	}

	public static class ClimberConstants {
		public static final int motorId = 20;
		public static final double p = 0.00075, i = 0, d = 0;
		public static final double maxMotorSpeed = 400;
		public static final double maxMotorPos = 360, minMotorPos = 0;
		public static final boolean isInverted = false;
		public static final int currentlimit = 90;
	}

	public static class ElevatorConstants {

		public static final double elevatorMaxSpeed = 1.0;
		public static final double elevatorMaxAcceleration = 0.5;

		public static final int elevatorMotorId = 30, elevatorMotorTwoId = 31;
		public static final double elevatorMotorP = 0.00015, elevatorMotorI = 0.0, elevatorMotorD = 0.0;
		public static final double motorS = 1.1, motorG = 1.2, motorV = 1.3;
		public static final boolean elevatorMotorInverted = true, elevatorMotorTwoInverted = true;

		public static final double maxDistance = 34.0, minDistance = 0;
		public static final double sourcePos = 0, levelFourPos = 0, LevelThreePos = 33.7;
		public static final double levelTwoPos = 15.2, homePos = 1.5;

		public static final int distanceSensorId = 39;

		
	}

	public static class SwerveConstants {
	  public static final double maxSpeedInFeet = 12;
	  public static final boolean isFieldOriented = true;
	}
}
