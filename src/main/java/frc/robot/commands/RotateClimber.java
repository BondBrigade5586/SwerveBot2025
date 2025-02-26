// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateClimber extends Command {

  private final ClimberSubsystem m_subsystem;
  private final Trigger m_forwardTrigger, m_backwardTrigger;
  /** Creates a new RotateClimber. */
  public RotateClimber(ClimberSubsystem subsystem, Trigger forwardTrigger, Trigger backwardTrigger) {
    m_subsystem = subsystem;
    m_forwardTrigger = forwardTrigger;
    m_backwardTrigger = backwardTrigger;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_forwardTrigger.getAsBoolean() == m_backwardTrigger.getAsBoolean()) {
      // m_subsystem.moveMotor(0);
      m_subsystem.stopMotor();
      return;
    }

    if (m_forwardTrigger.getAsBoolean()) {
      if (m_subsystem.getPosition() > ClimberConstants.maxMotorPos) {
      m_subsystem.stopMotor();
      // m_subsystem.moveMotor(0);
        return;
      }
      // m_subsystem.moveMotor(1);
      m_subsystem.moveMotorByPercent(0.7);
    } else if (m_backwardTrigger.getAsBoolean()) {
      if (m_subsystem.getPosition() < ClimberConstants.minMotorPos) {
      m_subsystem.stopMotor();
      // m_subsystem.moveMotor(0);
        return;
      }
      // m_subsystem.moveMotor(-1);
      m_subsystem.moveMotorByPercent(-0.7);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.moveMotorByPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !DriverStation.isJoystickConnected(OperatorConstants.kOperatorControllerPort);
    return false;
  }
}
