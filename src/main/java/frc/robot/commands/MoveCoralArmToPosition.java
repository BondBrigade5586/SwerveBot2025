// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveCoralArmToPosition extends Command {
  private CoralSubsystem m_subsystem;
  private double m_position, m_halfwayPosition, m_setPosition;
  private boolean m_commandIsFinished = false, m_withinBounds;
  /** Creates a new MoveCoralArmToPosition. */
  public MoveCoralArmToPosition(CoralSubsystem subsystem, double setPos) {
    m_halfwayPosition = (CoralConstants.maxPos - CoralConstants.minPos) / 2;
    m_setPosition = setPos;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_position = m_subsystem.getPosition();
    m_withinBounds = m_subsystem.withinBounds().getAsBoolean();

    if (!m_withinBounds) m_commandIsFinished = true;

    if (m_position > m_setPosition) {
      m_subsystem.setArmSpeed(-0.2);
    } else if (m_position < m_setPosition) {
      m_subsystem.setArmSpeed(0.2);
    } else {
      m_commandIsFinished = true;
    }

    if (m_position > m_halfwayPosition) {
      m_subsystem.setArmSpeed(-0.2);
    } else if (m_position < m_halfwayPosition) {
      m_subsystem.setArmSpeed(0.2);
    } else {
      m_commandIsFinished = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandIsFinished;
  }
}
