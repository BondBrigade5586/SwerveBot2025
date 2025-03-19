// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
  private PIDController m_xController, m_yController;
  private ProfiledPIDController m_thetaController;
  private RobotConfig m_config;
  
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

    try{
      m_config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    zeroGyro();

    m_xController = new PIDController(AutoConstants.autoXController, 0, 0);
    m_yController = new PIDController(AutoConstants.autoYController, 0, 0);
    m_thetaController = new ProfiledPIDController(
			AutoConstants.autoThetaControllerVal,
			0.0,
			0.0,
			AutoConstants.autoThetaControllerConstraints
		);

    AutoBuilder.configure(
      this::getPose,
      m_swerveDrive::resetOdometry,
      m_swerveDrive::getRobotVelocity, 
      (speed, feedForward) -> driveFieldOriented(speed, false), 
      new PPHolonomicDriveController(
        new PIDConstants(2.43, 0, 0), 
        new PIDConstants(2.477, 0, 0)
      ), 
      m_config, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);

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

  public void driveFieldOriented(ChassisSpeeds velocity, boolean trigger) {
    double speedMult = (trigger) ? 0.5 : 1;
    m_swerveDrive.driveFieldOriented(velocity.times(speedMult));
  }
  
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public Command driveFieldOrientedSlow(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get().times(0.5));
    });
  }

  public SendableChooser<Command> generateAutoChooser() {
    return AutoBuilder.buildAutoChooser();
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

	public SwerveControllerCommand generateCommand(Trajectory trajectory) {

		Consumer<SwerveModuleState[]> moduleStateConsumer = (states) -> {
			m_swerveDrive.setModuleStates(states, false);
		};

		return new SwerveControllerCommand(
				trajectory,
				m_swerveDrive::getPose,
				m_swerveDrive.kinematics,
        m_xController,
				m_yController,
				m_thetaController,
				moduleStateConsumer,
				this
		);
	}

  

  public SwerveInputStream getAngularVelocity(CommandXboxController controller) {
    double speedMult = (controller.rightTrigger().getAsBoolean()) ? 0.5 : 1;
    return SwerveInputStream.of(
      m_swerveDrive,
      () -> controller.getLeftY() * speedMult,
      () -> controller.getLeftX() * speedMult)
                                      .withControllerRotationAxis(() -> controller.getRightX() * -1)
                                      .deadband(Constants.OperatorConstants.controllerDeadband)
                                      .scaleTranslation(0.8)
                                      .allianceRelativeControl(true);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_swerveDrive.kinematics;
  }

  public SwerveModulePosition[] getModuleStates() {
    return m_swerveDrive.getModulePositions();
  }

  public Consumer<SwerveModuleState[]> getModuleStateConsumer() {
    return (states) -> {
			m_swerveDrive.setModuleStates(states, false);
		};
  }

  public Rotation2d getPitch() {
    m_swerveDrive.getPose();
    return m_swerveDrive.getPitch();
  }

  public Pose2d getPose() {
    return m_swerveDrive.getPose();
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
