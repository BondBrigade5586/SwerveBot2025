// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  Elevator m_Elevator;
  private Trigger m_forwardTrigger, m_backwardTrigger;
  /** Creates a new MoveElevator. */
  public MoveElevator(Elevator elevator, Trigger onTrigger, Trigger offTrigger) {
    m_Elevator = elevator;
    m_forwardTrigger = onTrigger;
    m_backwardTrigger = offTrigger;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Elevator.motorsAreNull()) return;
    if (m_forwardTrigger.getAsBoolean() == m_backwardTrigger.getAsBoolean()) {
      // m_Elevator.stop();
      m_Elevator.setElevatorVoltage(0.2);
      return;
    }

    if (m_forwardTrigger.getAsBoolean()) {
      m_Elevator.setElevatorSpeed(0.5);
    } else if (m_backwardTrigger.getAsBoolean()) {
      m_Elevator.setElevatorSpeed(-0.4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
