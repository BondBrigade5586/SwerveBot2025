package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
	Motor m_motor;
	DutyCycleEncoder m_throughBoreEncoder;
	public ClimberSubsystem(int motorId) {
		m_motor = new Motor(motorId, 20);
		m_motor.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motor.setInverted(AlgaeConstants.motorInverted);
		m_motor.burnConfig();
		m_throughBoreEncoder = new DutyCycleEncoder(0);

	}

	public BooleanSupplier withinBounds() {
		return () -> (getPosition() <= ClimberConstants.maxMotorPos && getPosition() >= ClimberConstants.minMotorPos);
	}

	public BooleanSupplier atSetPosition(double position) {
		return () -> (getPosition() < position + 0.1 && getPosition() > position - 0.1);
	}

	public double getPosition() {
		return m_throughBoreEncoder.get();
	}

	public Command moveToRotation(double rotation) {
		int rotationMult;
		if (rotation < 0) rotation += 360;
		rotation %= 360;
		rotationMult = (rotation < getPosition()) ? -1 : 1;
		return run(() -> {
			m_motor.setSpeed(ClimberConstants.maxMotorSpeed * 0.5 * rotationMult);
		}).onlyWhile(withinBounds());
	}

	public void moveMotor(int directionMult) {
		m_motor.setSpeed(ClimberConstants.maxMotorSpeed * 0.5 * directionMult);
	}

	public Command rotateCommand(double mult) {
		System.out.println("CLIMBER - rotate");

		final double effectiveMult = (mult >= 0) ? mult % 1 : (Math.abs(mult) % 1) * -1;
		return run(() -> {
			m_motor.setSpeed(ClimberConstants.maxMotorSpeed * effectiveMult);
		}).onlyWhile(withinBounds());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Position", getPosition());
	}
}
