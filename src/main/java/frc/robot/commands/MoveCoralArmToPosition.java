// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveCoralArmToPosition extends Command {
  private CoralSubsystem m_subsystem;
  private double m_position, m_setPosition;
  private boolean m_commandIsFinished = false, m_withinBounds;
  /** Creates a new MoveCoralArmToPosition. */
  public MoveCoralArmToPosition(CoralSubsystem subsystem, double setPos) {
    m_setPosition = setPos;
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  String m_string = "";

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_position = m_subsystem.getPosition();
    m_withinBounds = m_subsystem.withinBounds().getAsBoolean();
    // m_withinBounds = (m_position > CoralConstants.minPos && m_position < CoralConstants.maxPos);

    // m_commandIsFinished = m_withinBounds ? false : true;
    m_commandIsFinished = (Math.abs(m_position - m_setPosition) <= 7.5);

    //If too far up
    // if (m_position < CoralConstants.minPos) {
    //   if (m_position < m_setPosition) m_subsystem.setArmSpeed(-0.15);
    // } else if (m_position > CoralConstants.maxPos) {
    //   if (m_position > m_setPosition) m_subsystem.setArmSpeed(0.15);
    // } else if (m_position > CoralConstants.minPos && m_position < CoralConstants.maxPos) {
    //   if (m_position > m_setPosition) m_subsystem.setArmSpeed(0.175);
    //   if (m_position < m_setPosition) m_subsystem.setArmSpeed(-0.15);
    // } 

    if (m_position < CoralConstants.maxPos && m_position > CoralConstants.minPos) {
      if (m_position > m_setPosition) {
        if (m_withinBounds) m_subsystem.setArmSpeed(0.165);
      } else {
        m_subsystem.setArmSpeed(-0.1);
      }
    }

    // if (m_position > m_setPosition) {
    //   // if (m_position > CoralConstants.maxPos) return;
    //   if (m_position < CoralConstants.minPos) m_subsystem.setArmSpeed(0.15);
    //   m_string = "negative speed";
    // } else if (m_position < m_setPosition) {
    //   // if (m_position < CoralConstants.minPos) return;
    //   //b
    //   if (m_position > CoralConstants.maxPos) m_subsystem.setArmSpeed(-0.15);
    //   m_string = "positive speed";
    // } else {
    //   m_commandIsFinished = true;
    // }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandIsFinished;
    // return false;
  }
}
