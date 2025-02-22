package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
	Motor m_motor;
	DutyCycleEncoder m_throughBoreEncoder;
	public ClimberSubsystem(int motorId) {
		m_motor = new Motor(motorId, 20);
		m_motor.setPid(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
		m_motor.setInverted(ClimberConstants.isInverted);
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
		return m_throughBoreEncoder.get() * 360;
	}

	public void moveMotor(int directionMult) {
		m_motor.setPIDVelocity(ClimberConstants.maxMotorSpeed * 0.5 * directionMult);
	}

	public Command rotateCommand(double mult) {
		System.out.println("CLIMBER - rotate");
		

		final double effectiveMult = (mult >= 0) ? mult % 1 : (Math.abs(mult) % 1) * -1;
		return this.run(() -> {
			m_motor.setPIDVelocity(ClimberConstants.maxMotorSpeed * effectiveMult);
		}).onlyWhile(withinBounds()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
	}

	public Command jogMotorCommand(int speed) {
		// return this.runOnce(() -> moveMotor(speed));
		return this.run(() -> moveMotor(speed));
	}

	public Command stopMotorCommand() {
		return this.runOnce(() -> moveMotor(0));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Position", getPosition());
	}
}
