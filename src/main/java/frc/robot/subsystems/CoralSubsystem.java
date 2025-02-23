package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

class CoralSubsytem extends SubsystemBase {
	private AbsoluteEncoder m_pivotEncoder;
	private Motor m_driveMotor, m_pivotMotor;
	private SparkClosedLoopController m_pidController;

	CoralSubsytem(int wheelDriveMotorId, int pivotMotorId) {
		
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
		m_driveMotor.setPIDVelocity(CoralConstants.driveMotorMaxSpeed * speedMult);
	}

	void outtake(double speedMult) {
		m_driveMotor.setPIDVelocity(CoralConstants.driveMotorMaxSpeed * speedMult * -1);
	}

	public void setArmSpeed(double speedMult) {
		m_pivotMotor.setPIDVelocity(CoralConstants.angleMotorMaxSpeed * speedMult);
	}
	
}
