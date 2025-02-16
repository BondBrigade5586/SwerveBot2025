package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class ClimberSubsystem extends SubsystemBase{
	Motor m_motor;
	DutyCycleEncoder m_throughBoreEncoder;
	ClimberSubsystem(int motorId) {
		m_motor = new Motor(motorId, 20);
		m_motor.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motor.setInverted(AlgaeConstants.motorInverted);
		m_motor.burnConfig();
		m_throughBoreEncoder = new DutyCycleEncoder(0);

	}

	public double getPosition() {
		return m_throughBoreEncoder.get();
	}
}
