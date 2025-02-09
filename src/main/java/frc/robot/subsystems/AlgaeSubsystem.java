package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.AlgaeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class AlgaeSubsystem extends SubsystemBase {
	private Motor m_motor, m_motorTwo;

	AlgaeSubsystem(int motorId, int motorTwoId) {
		
		//private SparkMax m_motor, m_motorTwo;
		//m_motor = new SparkMax(motorId, MotorType.kBrushless);
		//m_motorTwo = new SparkMax(motorTwoId, MotorType.kBrushless);
		
		m_motor = new Motor(motorId, 20);
		m_motor.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motor.setInverted(AlgaeConstants.motorInverted);
		m_motor.burnConfig();

		m_motorTwo = new Motor(motorTwoId, 20);
		m_motorTwo.setPid(AlgaeConstants.p, AlgaeConstants.i, AlgaeConstants.d);
		m_motorTwo.setInverted(AlgaeConstants.motorTwoInverted);
		m_motorTwo.burnConfig();
	}

	void intake(double speedMult) {
		m_motor.setSpeed(speedMult * AlgaeConstants.maxSpeed);
		m_motorTwo.setSpeed(speedMult * AlgaeConstants.maxSpeed);
	}

	void outtake(double speedMult) {
		m_motor.setSpeed(-speedMult * AlgaeConstants.maxSpeed);
		m_motorTwo.setSpeed(-speedMult * AlgaeConstants.maxSpeed);
	}

}
