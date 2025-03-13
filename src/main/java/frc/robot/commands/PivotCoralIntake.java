// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCoralIntake extends Command {
  private final CoralSubsystem m_subsytem;
  Trigger m_forwardTrigger, m_backwardTrigger;
  boolean m_isPivoting = false;
  CommandXboxController m_controller;
  enum InputState {
    JOYSTICK_UP,
    JOYSTICK_DOWN,
    NULL
  }
  InputState m_inputState;
  /** Creates a new PivotCoralIntake. */
  public PivotCoralIntake(CoralSubsystem coralSubsytem, CommandXboxController controller) {
    m_controller = controller;
    m_forwardTrigger = new Trigger(() -> m_controller.getRightY() > 0.2);
    m_backwardTrigger = new Trigger(() -> m_controller.getRightY() < -0.2);
    m_subsytem = coralSubsytem;
    addRequirements(m_subsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_inputState = processInput();
    switch (m_inputState) {
      case NULL:
        if (m_subsytem.getPosition() > CoralConstants.minPos) {
          m_subsytem.setArmVoltage(0.4);
        } else {
          m_subsytem.stopArm();
        }
        break;
      case JOYSTICK_UP:
        m_subsytem.setArmSpeed(0.2);
        break;
      case JOYSTICK_DOWN:
        m_subsytem.setArmSpeed(-0.2);
        break;
      default:
        break;
    }
    // if ( m_forwardTrigger.getAsBoolean() == m_backwardTrigger.getAsBoolean() || 
    //     (m_forwardTrigger.getAsBoolean() && m_subsytem.getPosition() > CoralConstants.maxPos) || 
    //     (m_backwardTrigger.getAsBoolean() && m_subsytem.getPosition() < CoralConstants.minPos)
    //    ) {

    //     if (m_subsytem.getPosition() > CoralConstants.minPos) {
    //       m_subsytem.setArmVoltage(-0.4);
    //     } else {
    //       m_subsytem.stopArm();
    //     }
    // } else if (m_forwardTrigger.getAsBoolean()) {
    //   m_subsytem.setArmSpeed(0.2);
    // } else if (m_backwardTrigger.getAsBoolean()) {
    //   m_subsytem.setArmSpeed(-0.2);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private InputState processInput() {
    if ( m_forwardTrigger.getAsBoolean() == m_backwardTrigger.getAsBoolean() || 
        (m_forwardTrigger.getAsBoolean() && m_subsytem.getPosition() > CoralConstants.maxPos) || 
        (m_backwardTrigger.getAsBoolean() && m_subsytem.getPosition() < CoralConstants.minPos)
       ) {
        return InputState.NULL;
    } else if (m_forwardTrigger.getAsBoolean()) {
      return InputState.JOYSTICK_UP;
    } else if (m_backwardTrigger.getAsBoolean()) {
      return InputState.JOYSTICK_DOWN;
    }
    return InputState.NULL;
  }

  @Override
  public ParallelRaceGroup withTimeout(double seconds) {
    return super.withTimeout(3);
  }
}
