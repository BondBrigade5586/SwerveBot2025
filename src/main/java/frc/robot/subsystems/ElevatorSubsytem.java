package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

class ElevatorSubsystem extends SubsystemBase {
	private Motor m_elevatorMotor, m_elevatorMotorTwo, m_trolleyMotor;

	ElevatorSubsystem(int motorId, int motorTwoId, int trolleyMotorId) {
		
		m_elevatorMotor = new Motor(motorId, 20);
		m_elevatorMotor.setPid(ElevatorConstants.elevatorMotorP, ElevatorConstants.elevatorMotorI, ElevatorConstants.elevatorMotorD);
		m_elevatorMotor.setInverted(ElevatorConstants.elevatorMotorInverted);
		m_elevatorMotor.burnConfig();

		m_elevatorMotorTwo = new Motor(motorTwoId, 20);
		m_elevatorMotorTwo.setPid(ElevatorConstants.elevatorMotorTwoP, ElevatorConstants.elevatorMotorTwoI, ElevatorConstants.elevatorMotorTwoD);
		m_elevatorMotorTwo.setInverted(ElevatorConstants.elevatorMotorTwoInverted);
		m_elevatorMotorTwo.burnConfig();

		m_trolleyMotor = new Motor(trolleyMotorId, 20);
		m_trolleyMotor.setPid(ElevatorConstants.trolleyMotorP, ElevatorConstants.trolleyMotorI, ElevatorConstants.trolleyMotorD);
		m_trolleyMotor.setInverted(ElevatorConstants.trolleyMotorInverted);
		m_trolleyMotor.burnConfig();

	}

	void setElevatorSpeed(double speedMult) {
		m_elevatorMotor.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
		m_elevatorMotorTwo.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
	}

	void setTrolleySpeed(double speedMult) {
		m_trolleyMotor.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
	}

}
