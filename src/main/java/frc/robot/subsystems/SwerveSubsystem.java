// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.autonomous.AutoTrajectory;

public class SwerveSubsystem extends SubsystemBase {

  private File m_file;
  private double m_maxSpeed;
  private SwerveDrive m_swerveDrive;
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    m_maxSpeed = Units.feetToMeters(Constants.SwerveConstants.maxSpeedInFeet);
    m_file = new File(Filesystem.getDeployDirectory(), "swerve");

    //                               Verbosity of ShuffleBoard information
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerveDrive = new SwerveParser(m_file).createSwerveDrive(m_maxSpeed);
    } catch (Exception err) {
      throw new RuntimeException(err);
    }

    zeroGyro();

  }

	public Trigger createDistanceTrigger(double x, double y, double r, double margin) {
		return new Trigger(isNearPoint(x, y, r, margin));
	}

    /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      m_swerveDrive.drive(m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      m_swerveDrive.getOdometryHeading().getRadians(),
                                                                      m_swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      m_swerveDrive.drive(new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
                                            translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * m_swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }
  
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

	public SwerveControllerCommand generateCommand(AutoTrajectory trajectory) {

		Consumer<SwerveModuleState[]> moduleStateConsumer = (states) -> {
			m_swerveDrive.setModuleStates(states, false);
		};

		return new SwerveControllerCommand(
				trajectory.getTrajectory(),
				m_swerveDrive::getPose,
				m_swerveDrive.kinematics,
        trajectory.getXController(),
				trajectory.getYController(),
				trajectory.getThetaController(),
				moduleStateConsumer,
				this
		);
	}

  public SwerveInputStream getAngularVelocity(CommandXboxController controller) {
    return SwerveInputStream.of(
      m_swerveDrive,
      () -> controller.getLeftY() * -1,
      () -> controller.getLeftX() * -1)
                                      .withControllerRotationAxis(controller::getRightX)
                                      .deadband(Constants.OperatorConstants.controllerDeadband)
                                      .scaleTranslation(0.8)
                                      .allianceRelativeControl(true);
  }

  public SwerveModulePosition[] getModuleStates() {
    return m_swerveDrive.getModulePositions();
  }

  public Rotation2d getPitch() {
    return m_swerveDrive.getPitch();
  }

	public BooleanSupplier isNearPoint(double x, double y, double rotation, double margin) {
		//Makes sure trigger never fires in auto.
		if (!DriverStation.isAutonomous()) return () -> false;

		Translation2d currentPoint = m_swerveDrive.getPose().getTranslation();
		Translation2d paramPoint = new Translation2d(x, y);

		return () -> currentPoint.getDistance(paramPoint) <= margin;
	}

  public void zeroGyro() {
    m_swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
		SmartDashboard.putString("Swerve Pose", "X: " + m_swerveDrive.getPose().getX() + ", Y: " + m_swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Swerve Yaw", m_swerveDrive.getGyro().getRawRotation3d().getAngle());
    return;
  }
}
