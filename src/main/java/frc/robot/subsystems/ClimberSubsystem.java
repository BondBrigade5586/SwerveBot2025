package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

	private Motor m_motor;
	private DutyCycleEncoder m_throughBoreEncoder;

	public ClimberSubsystem(int motorId) {
		m_motor = new Motor(motorId, ClimberConstants.currentlimit, IdleMode.kBrake);
		m_motor.setPid(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
		m_motor.setInverted(ClimberConstants.isInverted);
		m_motor.burnConfig();
		m_throughBoreEncoder = new DutyCycleEncoder(0);
	}

	public BooleanSupplier isAtSetPosition(double position) {
		return () -> (getPosition() < position + 0.1 && getPosition() > position - 0.1);
	}

	public void cancelCurrentCommand() {
		Command currentCommand = this.getCurrentCommand();
		if (currentCommand == null) return;
		currentCommand.cancel();
	}

	public double getPosition() {
		return m_throughBoreEncoder.get() * 360;
	}
	
	public BooleanSupplier inputIsValid(int speed) {
		if (getPosition() >= ClimberConstants.maxMotorPos && speed == 1) return () -> false;
		if (getPosition() <= ClimberConstants.minMotorPos && speed == -1) return () -> false;
		return () -> true;
	}

	public Command jogMotorCommand(int speed) {
		return this.run(() -> {
			moveMotor(speed);
		});
	}

	public void moveMotor(int directionMult) {
		
		if (getPosition() >= ClimberConstants.maxMotorPos && directionMult == 1) return;
		if (getPosition() <= ClimberConstants.minMotorPos && directionMult == -1) return;
		
		m_motor.setPIDVelocity(ClimberConstants.maxMotorSpeed * directionMult);
	}

	public void moveMotorByPercent(double percent) {
		
		if (getPosition() >= ClimberConstants.maxMotorPos && percent >= 0) return;
		if (getPosition() <= ClimberConstants.minMotorPos && percent <= 0) return;

		m_motor.setSpeed(percent);
	}

	public Command rotateCommand(double mult) {
		final double effectiveMult = (mult >= 0) ? mult % 1 : (Math.abs(mult) % 1) * -1;
		return this.run(() -> {
			m_motor.setPIDVelocity(ClimberConstants.maxMotorSpeed * effectiveMult);
		});
	}

	public Command stopMotorCommand() {
		// return this.runOnce(() -> moveMotor(0));
		return this.run(() -> m_motor.setPidPosition(m_motor.getEncoder().getPosition()));
	}

	public void stopMotor() {
		m_motor.setPidPosition(m_motor.getEncoder().getPosition());
	}

	public BooleanSupplier isWithinBounds() {
		return () -> (getPosition() <= ClimberConstants.maxMotorPos && getPosition() >= ClimberConstants.minMotorPos);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Position", getPosition());
		SmartDashboard.putBoolean("Climber in bounds", isWithinBounds().getAsBoolean());
	}
}

