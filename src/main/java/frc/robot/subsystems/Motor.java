package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

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

	public void burnConfig() {
		// Test PersistMode and ResetMode
		m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

	public void setCurrentLimit(int currentLimit) {
		m_motorConfig.smartCurrentLimit(currentLimit);
	}

	public void setInverted(boolean isInverted) {
		m_motorConfig.inverted(isInverted);
	}

	public void setPid(double p, double i, double d) {
		m_motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
		pid(p, i, d);
	}

	public void setPIDVelocity(double velocity) {
		m_closedLoopController.setReference(velocity, ControlType.kVelocity);
	}

	public void setSpeed(double speed) {
		m_motor.set(speed);
	}

}
