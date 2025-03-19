// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoTrajectory;
import frc.robot.commands.MoveCoralArmToPosition;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.PivotCoralIntake;
import frc.robot.commands.RotateClimber;
import swervelib.SwerveInputStream;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
	private AutoTrajectory m_autoTrajectory;
	private SwerveControllerCommand m_driveForwardCommand;
  private final ClimberSubsystem m_climberSubsystem;
  private final Elevator m_elevatorSubsystem;
  private final AlgaeSubsystem m_algaeSubsystem;
  private final CoralSubsystem m_coralSubsystem;
	Trigger m_autoTrigger;
  private final SendableChooser<Command> m_autoChooser;

  SwerveInputStream m_angularVelocity;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_climberSubsystem = new ClimberSubsystem(ClimberConstants.motorId);
    m_elevatorSubsystem = new Elevator(ElevatorConstants.elevatorMotorId, ElevatorConstants.elevatorMotorTwoId);
    m_swerveSubsystem = new SwerveSubsystem();
    m_algaeSubsystem = new AlgaeSubsystem(AlgaeConstants.motorId, AlgaeConstants.motorTwoId);
    m_coralSubsystem = new CoralSubsystem(CoralConstants.motorId, CoralConstants.pivotMotorId);

    m_autoChooser = m_swerveSubsystem.generateAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    configureBindings();

		//This command will trigger when the robot is close to the defined point TESTING!!
		m_autoTrigger = m_swerveSubsystem.createDistanceTrigger(1, 1, 1, 0.1);
		m_autoTrigger.onTrue(new SequentialCommandGroup(
														 //Rotate to right angle
														 new InstantCommand(),
														 //Release coral
														 new InstantCommand()));

		m_autoTrajectory = new AutoTrajectory(AutoConstants.maxVelocity, AutoConstants.maxAcceleration);
    // m_autoTrajectory.modifyStartPosition(new Pose2d(Units.inchesToMeters(297.5), Units.inchesToMeters(241.44), Rotation2d.fromDegrees(0)));
		// m_autoTrajectory.translate(-1 * Units.inchesToMeters(88), 0.0);
		// m_autoTrajectory.generateTrajectory();
		// m_driveForwardCommand = m_swerveSubsystem.generateCommand(m_autoTrajectory);

  }

  private boolean intakeCommandShouldRun(CommandXboxController controller) {
    return !(Math.abs(controller.getLeftY()) > 0.2 || Math.abs(controller.getLeftX()) > 0.2);
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
    Command fieldOrientedDriveSlow = m_swerveSubsystem.driveFieldOrientedSlow(m_angularVelocity);
    Command rotateArm = new RotateClimber(
      m_climberSubsystem,
      m_operatorController.povUp(),
      m_operatorController.povDown()
    );

    Command moveElevator = new MoveElevator(
      m_elevatorSubsystem,
      new Trigger(() -> m_operatorController.getRightY() < -0.2),
      new Trigger(() -> m_operatorController.getRightY() > 0.2)
    );

    Command moveCoralIntake = new PivotCoralIntake(
      m_coralSubsystem,
      new Trigger(() -> m_operatorController.getLeftY() < -0.2),
      new Trigger(() -> m_operatorController.getLeftY() > 0.2)
    );

    Command moveCoralIntakeToHome = new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.homePos);
    Command moveCoralIntakeToLevelTwo = new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.levelTwoPos);
    Command moveCoralIntakeToLevelThree = new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.LevelThreePos);
    Command moveCoralIntakeToSource = new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.sourcePos);

    Command moveElevatorToHome = new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.homePos);
    Command moveElevatorToLevelTwo = new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.levelTwoPos);
    Command moveElevatorToLevelThree = new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.LevelThreePos);
    Command moveElevatorToSource = new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.sourcePos);

    m_driverController.y().onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.zeroGyro()));
    m_driverController.rightTrigger().whileTrue(fieldOrientedDriveSlow);

    //TODO: Make Commands more consistent.
    m_operatorController.b().onTrue(new SequentialCommandGroup(
      new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.homePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController)),
      new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.homePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController))

    ));

    m_operatorController.x().onTrue(new SequentialCommandGroup(
      new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.levelTwoPos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController)),
      new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.levelTwoPos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController))
    ));

    m_operatorController.y().onTrue(new SequentialCommandGroup(
      new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.LevelThreePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController)),
      new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.LevelThreePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController))
    ).withTimeout(3));

    m_operatorController.a().onTrue(new SequentialCommandGroup(
      new MoveElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.sourcePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController)),
      new MoveCoralArmToPosition(m_coralSubsystem, CoralConstants.sourcePos).onlyWhile(() -> intakeCommandShouldRun(m_operatorController))
    ));

    m_operatorController.leftBumper().onTrue(m_algaeSubsystem.intakeCommand(0)).onFalse(m_algaeSubsystem.stopCommand());
    m_operatorController.leftTrigger().onTrue(m_algaeSubsystem.outtakeCommand(0)).onFalse(m_algaeSubsystem.stopCommand());
    
    m_operatorController.rightBumper().onTrue(m_coralSubsystem.intakeCommand(0.7)).onFalse(m_coralSubsystem.stopCommand());
    m_operatorController.rightTrigger().onTrue(m_coralSubsystem.outtakeCommand(0.7)).onFalse(m_coralSubsystem.stopCommand());

    m_swerveSubsystem.setDefaultCommand(fieldOrientedDrive);
    m_climberSubsystem.setDefaultCommand(rotateArm);
    m_elevatorSubsystem.setDefaultCommand(moveElevator);
    m_coralSubsystem.setDefaultCommand(moveCoralIntake);

    //Register commands for PathPlanner
    NamedCommands.registerCommand("Move Coral Intake To L4", moveCoralIntakeToLevelTwo);
    NamedCommands.registerCommand("Move Coral Intake To L3", moveCoralIntakeToLevelThree);
    NamedCommands.registerCommand("Move Coral Intake To Home", moveCoralIntakeToHome);
    NamedCommands.registerCommand("Move Coral Intake To Source", moveCoralIntakeToSource);

    // NamedCommands.registerCommand("Move Elevator To L4", moveElevatorToLevelFour);
    NamedCommands.registerCommand("Move Elevator To L3", moveElevatorToLevelThree);
    NamedCommands.registerCommand("Move Elevator To Home", moveElevatorToHome);
    NamedCommands.registerCommand("Move Elevator To Source", moveElevatorToSource);

  }

  public void resetElevatorEncoders() {
    m_elevatorSubsystem.resetEncoders();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    // return new PathPlannerAuto("Test Auto");
  }
}
