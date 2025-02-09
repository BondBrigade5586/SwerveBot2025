package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

class CoralSubsytem extends SubsystemBase {
	AbsoluteEncoder m_pivotEncoder;
	Motor m_driveMotor, m_pivotMotor;
	SparkClosedLoopController m_pidController;

	CoralSubsytem(int wheelDriveMotorId, int pivotMotorId) {
		//m_wheelDriveMotor = new SparkMax(wheelDriveMotorId, MotorType.kBrushless);
		//m_pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
		//SparkMax m_wheelDriveMotor, m_pivotMotor;
		
		m_driveMotor = new Motor(wheelDriveMotorId, 20);
		m_driveMotor.setPid(CoralConstants.wheelDriveP, CoralConstants.wheelDriveI, CoralConstants.wheelDriveP);
		m_driveMotor.setInverted(CoralConstants.wheelDriveInverted);
		m_driveMotor.burnConfig();

		m_pivotMotor = new Motor(pivotMotorId, 20);
		m_pivotMotor.setPid(CoralConstants.angleMotorP, CoralConstants.angleMotorI, CoralConstants.angleMotorD);
		m_pivotMotor.setInverted(CoralConstants.angleMotorInverted);
		m_pivotMotor.burnConfig();
		m_pidController = m_pivotMotor.getMotor().getClosedLoopController();
		m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder();
	}

	void intake(double speedMult) {
		m_driveMotor.setSpeed(speedMult * CoralConstants.driveMotorMaxSpeed);
	}

	void setWheelMotorSpeed(double speedMult) {
		m_driveMotor.setSpeed(speedMult * CoralConstants.driveMotorMaxSpeed);
	}

	public void setArmSpeed(double speedMult) {
		m_pivotMotor.setSpeed(speedMult * CoralConstants.angleMotorMaxSpeed);
	}

	public void setArmPosition(double setPosition) {
		m_pidController.setReference(setPosition, ControlType.kPosition);
	}

	public void holdArmPosition() {
		m_pidController.setReference(m_pivotEncoder.getPosition(), ControlType.kPosition);
	}
	
}
