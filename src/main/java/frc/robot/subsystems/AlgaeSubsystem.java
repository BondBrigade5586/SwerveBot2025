package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
	private Motor m_motor, m_motorTwo;

	public AlgaeSubsystem(int motorId, int motorTwoId) {
		
		m_motor = new Motor(motorId, 20, IdleMode.kBrake);
		m_motor.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motor.setInverted(AlgaeConstants.motorInverted);
		m_motor.burnConfig();

		m_motorTwo = new Motor(motorTwoId, 20, IdleMode.kBrake);
		m_motorTwo.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motorTwo.setInverted(AlgaeConstants.motorTwoInverted);
		m_motorTwo.burnConfig();
	}

	void intake(double speedMult) {
		m_motor.setPIDVelocity(speedMult * AlgaeConstants.maxSpeed);
		m_motorTwo.setPIDVelocity(speedMult * AlgaeConstants.maxSpeed);
	}

	public boolean motorsAreNull() {
		return (m_motor.getMotor().getFaults().can || m_motorTwo.getMotor().getFaults().can);
	}

	void outtake(double speedMult) {
		m_motor.setPIDVelocity(speedMult * AlgaeConstants.maxSpeed * -1);
		m_motorTwo.setPIDVelocity(speedMult * AlgaeConstants.maxSpeed * -1);
	}

	public Command intakeCommand(double speedMult) {
		return this.run(() -> {
			m_motor.setSpeed(speedMult);
			m_motorTwo.setSpeed(speedMult);
		});
	}

	public Command outtakeCommand(double speedMult) {
		return this.run(() -> {
			m_motor.setSpeed(0.3 * -1);
			m_motorTwo.setSpeed(0.3 * -1);
		});
	}

	public Command stopCommand() {
		return this.runOnce(() -> {
			m_motor.setSpeed(0);
			m_motorTwo.setSpeed(0);
		});
	}

}
