// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {
  private Elevator m_subsystem;
  private double m_setPosition, m_currentPosition, m_speed = 0;
  private boolean m_commandShouldFinish;
  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Elevator elevatorSubsystem, double position) {
    m_setPosition = position;
    m_subsystem = elevatorSubsystem;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPosition = m_subsystem.getElevatorDistance();
    m_commandShouldFinish = (!m_subsystem.withinBounds().getAsBoolean());
    m_speed = (m_currentPosition < m_setPosition) ? 0.2 : -0.2;
    
    if (m_currentPosition > ElevatorConstants.minDistance && m_currentPosition < ElevatorConstants.maxDistance) {
      // if (m_currentPosition < m_setPosition) m_subsystem.setElevatorSpeed(0.2);
      m_subsystem.setElevatorSpeed(m_speed);
      if (Math.abs(m_currentPosition - m_setPosition) < 0.2) m_commandShouldFinish = true;
    } else if (m_currentPosition > ElevatorConstants.maxDistance) {
      if (m_currentPosition > m_setPosition) m_subsystem.setElevatorSpeed(m_speed);
    } else if (m_currentPosition < ElevatorConstants.minDistance) {
      if (m_currentPosition < m_setPosition) m_subsystem.setElevatorSpeed(m_speed);
    }

    // if (m_currentPosition > m_setPosition) {
    //   m_subsystem.setElevatorSpeed(-0.2);
    // } else if (m_currentPosition < m_setPosition) {
    //   m_subsystem.setElevatorSpeed(0.2);
    // } else {
    //   m_commandShouldFinish = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandShouldFinish;
  }

  @Override
  public ParallelRaceGroup withTimeout(double seconds) {
      return super.withTimeout(3);
  }
}
