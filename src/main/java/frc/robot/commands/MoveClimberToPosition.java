package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.BooleanSupplier;

class MoveClimberToPosition extends Command {
	private final ClimberSubsystem m_climberSubsystem;
	private double m_position;
	private int m_directionMult;

	MoveClimberToPosition(ClimberSubsystem climberSubsystem, double pos) {
		m_position = pos;
		m_climberSubsystem = climberSubsystem;
		addRequirements(m_climberSubsystem);
		m_directionMult = (m_climberSubsystem.getPosition() < pos) ? 1 : -1;
		//this.onlyWhile(m_climberSubsystem.withinBounds());
	}

	@Override
	public void execute() {
		m_climberSubsystem.moveMotor(m_directionMult);
	}

	@Override
	public boolean isFinished() {
		return (m_climberSubsystem.atSetPosition(m_position).getAsBoolean() | !m_climberSubsystem.withinBounds().getAsBoolean());
	}
}
