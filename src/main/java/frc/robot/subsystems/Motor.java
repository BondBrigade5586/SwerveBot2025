package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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

class Motor {

	SparkMax m_motor;
	SparkMaxConfig m_motorConfig;
	RelativeEncoder m_relativeEncoder;
	AbsoluteEncoder m_absoluteEncoder;
	SparkClosedLoopController m_closedLoopController;
	double m_p, m_i, m_d;

	Motor(int motorId, int currentLimit) {
		m_motor = new SparkMax(motorId, MotorType.kBrushless);
		m_motorConfig = new SparkMaxConfig();
		// m_motorConfig.idleMode(IdleMode.kBrake);
		m_motorConfig.smartCurrentLimit(currentLimit);
		m_closedLoopController = m_motor.getClosedLoopController();
	}

	AbsoluteEncoder getAbsoluteEncoder() {
		return m_motor.getAbsoluteEncoder();
	}

	RelativeEncoder getEncoder() {
		return m_motor.getEncoder();
	}

	RelativeEncoder getAlternateEncoder() {
		return m_motor.getAlternateEncoder();
	}

	SparkMax getMotor() {
		return m_motor;
	}

	SparkMaxConfig getConfig() {
		return m_motorConfig;
	}

	public void setPid(double p, double i, double d) {
		m_motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
		pid(p, i, d);
	}

	public void setInverted(boolean isInverted) {
		m_motorConfig.inverted(isInverted);
	}

	public void setCurrentLimit(int currentLimit) {
		m_motorConfig.smartCurrentLimit(currentLimit);
	}

	public void setPIDVelocity(double velocity) {
		m_closedLoopController.setReference(velocity, ControlType.kVelocity);
	}

	public void burnConfig() {
		// Test PersistMode and ResetMode
		m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public void setSpeed(double speed) {
		m_motor.set(speed);
	}

}
