// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCoralIntake extends Command {
  private final CoralSubsystem m_subsytem;
  Trigger m_forwardTrigger, m_backwardTrigger;
  /** Creates a new PivotCoralIntake. */
  public PivotCoralIntake(CoralSubsystem coralSubsytem, Trigger forwardTrigger, Trigger backwardTrigger) {
    m_forwardTrigger = forwardTrigger;
    m_backwardTrigger = backwardTrigger;
    m_subsytem = coralSubsytem;
    addRequirements(m_subsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_forwardTrigger.getAsBoolean() == m_forwardTrigger.getAsBoolean() || (m_forwardTrigger.getAsBoolean() && m_subsytem.getPosition() > CoralConstants.maxPos) || (m_backwardTrigger.getAsBoolean() && m_subsytem.getPosition() < CoralConstants.minPos)) {
      m_subsytem.setArmSpeed(0);
      return;
    }
    if (m_forwardTrigger.getAsBoolean()) {
      m_subsytem.setArmSpeed(0.1);
    } else {
      m_subsytem.setArmSpeed(-0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
