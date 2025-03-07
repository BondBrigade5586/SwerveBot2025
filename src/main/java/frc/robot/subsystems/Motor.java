package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;

class Motor {

	private SparkMax m_motor;
	private SparkMaxConfig m_motorConfig;
	private SparkClosedLoopController m_closedLoopController;

	Motor(int motorId, int currentLimit) {
		m_motor = new SparkMax(motorId, MotorType.kBrushless);
		m_motorConfig = new SparkMaxConfig();
		// m_motorConfig.idleMode(IdleMode.kBrake);
		m_motorConfig.smartCurrentLimit(currentLimit);
		m_closedLoopController = m_motor.getClosedLoopController();
	}

	Motor(int motorId, int currentLimit, IdleMode brakeMode) {
		m_motor = new SparkMax(motorId, MotorType.kBrushless);
		m_motorConfig = new SparkMaxConfig();
		m_motorConfig.idleMode(brakeMode);
		m_motorConfig.smartCurrentLimit(currentLimit);
		m_closedLoopController = m_motor.getClosedLoopController();
	}

	public void burnConfig() {
		// Test PersistMode and ResetMode
		m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public void copyConfig(int motorId) {
		m_motorConfig.follow(motorId);
	}

	public void copyConfig(SparkBase motor) {
		m_motorConfig.follow(motor);
	}

	public AbsoluteEncoder getAbsoluteEncoder() {
		return m_motor.getAbsoluteEncoder();
	}

	public RelativeEncoder getAlternateEncoder() {
		return m_motor.getAlternateEncoder();
	}

	public SparkMaxConfig getConfig() {
		return m_motorConfig;
	}

	public RelativeEncoder getEncoder() {
		return m_motor.getEncoder();
	}

	public SparkMax getMotor() {
		return m_motor;
	}

	public void setAllowedClosedLoopError(double error) {
		m_motorConfig.closedLoop.maxMotion.allowedClosedLoopError(error);
	}

	public void setCurrentLimit(int currentLimit) {
		m_motorConfig.smartCurrentLimit(currentLimit);
	}

	public void setFeedbackSensor(ClosedLoopConfig.FeedbackSensor encoder) {
		m_motorConfig.closedLoop.feedbackSensor(encoder);
	}

	public void setInverted(boolean isInverted) {
		m_motorConfig.inverted(isInverted);
	}

	public void setMaxAcceleration(double acceleration) {
		m_motorConfig.closedLoop.maxMotion.maxAcceleration(acceleration);
	}

	public void setMaxVelocity(double velocity) {
		m_motorConfig.closedLoop.maxMotion.maxVelocity(velocity);
	}

	public void setPid(double p, double i, double d) {
		m_motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
		pid(p, i, d);
	}

	public void setPidPosition(double position) {
		m_motor.getClosedLoopController().setReference(position, ControlType.kPosition);
	}

	public void setPIDVelocity(double velocity) {
		m_closedLoopController.setReference(velocity, ControlType.kVelocity);
	}

	public void setSpeed(double speed) {
		m_motor.set(speed);
	}

	public void setVoltage(double voltage) {
		m_motor.setVoltage(voltage);
	}

	public void stop() {
		m_closedLoopController.setReference(m_motor.getEncoder().getPosition(), ControlType.kPosition);
	}

	public void controlledVelocity() {
		m_motor.getClosedLoopController().setReference(2, ControlType.kCurrent);
	}

}
