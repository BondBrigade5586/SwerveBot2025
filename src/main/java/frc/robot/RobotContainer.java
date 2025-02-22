// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoTrajectory;
import frc.robot.commands.RotateClimber;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  // private final Joystick m_operatorController;
  private final CommandXboxController m_operatorController;
  // private final JoystickButton m_operatorLeftBumper, m_operatorRightBumper;
  public final SwerveSubsystem m_swerveSubsystem;
	private final AutoTrajectory m_autoTrajectory;
	private SwerveControllerCommand m_driveForwardCommand;
  private final ClimberSubsystem m_climberSubsystem;
	Trigger m_autoTrigger;

  SwerveInputStream m_angularVelocity;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);


    m_swerveSubsystem = new SwerveSubsystem();
    m_climberSubsystem = new ClimberSubsystem(ClimberConstants.motorId);
    configureBindings();

		//This command will trigger when the robot is close to the defined point TESTING!!
		m_autoTrigger = m_swerveSubsystem.distanceTrigger(1, 1, 1, 0.1);
		m_autoTrigger.onTrue(new SequentialCommandGroup(
														 //Rotate to right angle
														 new InstantCommand(),
														 //Release coral
														 new InstantCommand()));

		m_autoTrajectory = new AutoTrajectory(1.0, 1.0);
		m_autoTrajectory.pushPosition(0.0, -2.0, 0.0);
		m_autoTrajectory.generateTrajectory();
		m_driveForwardCommand = m_swerveSubsystem.generateCommand(m_autoTrajectory);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_angularVelocity = m_swerveSubsystem.getAngularVelocity(m_driverController);
    Command fieldOrientedDrive = m_swerveSubsystem.driveFieldOriented(m_angularVelocity);
    Command rotateArm = new RotateClimber(m_climberSubsystem, m_operatorController.rightBumper(), m_operatorController.leftBumper());

    m_swerveSubsystem.setDefaultCommand(fieldOrientedDrive);
    m_climberSubsystem.setDefaultCommand(rotateArm);

    //Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
    //    () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.controllerDeadband),
    //    () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.controllerDeadband),
    //    () -> m_driverController.getRightX(),
    //    () -> m_driverController.getRightY());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_driveForwardCommand;
  }
}
