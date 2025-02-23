package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

class ElevatorSubsystem extends SubsystemBase {
	private Motor m_elevatorMotor, m_elevatorMotorTwo;

	ElevatorSubsystem(int motorId, int motorTwoId, int trolleyMotorId) {
		
		m_elevatorMotor = new Motor(motorId, 20);
		m_elevatorMotor.setPid(ElevatorConstants.elevatorMotorP, ElevatorConstants.elevatorMotorI, ElevatorConstants.elevatorMotorD);
		m_elevatorMotor.setInverted(ElevatorConstants.elevatorMotorInverted);
		m_elevatorMotor.burnConfig();

		m_elevatorMotorTwo = new Motor(motorTwoId, 20);
		m_elevatorMotorTwo.setPid(ElevatorConstants.elevatorMotorTwoP, ElevatorConstants.elevatorMotorTwoI, ElevatorConstants.elevatorMotorTwoD);
		m_elevatorMotorTwo.setInverted(ElevatorConstants.elevatorMotorTwoInverted);
		m_elevatorMotorTwo.burnConfig();

	}

	void setElevatorSpeed(double speedMult) {
		m_elevatorMotor.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
		m_elevatorMotorTwo.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
	}

}
