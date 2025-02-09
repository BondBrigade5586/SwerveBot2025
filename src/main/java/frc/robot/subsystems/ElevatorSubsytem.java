package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

class ElevatorSubsystem extends SubsystemBase {
	private Motor m_elevatorMotor, m_elevatorMotorTwo, m_trolleyMotor;
	private SparkClosedLoopController m_elevatorMotorController, m_elevatorMotorControllerTwo;
	private SparkClosedLoopController m_trolleyMotorController;

	ElevatorSubsystem(int motorId, int motorTwoId, int trolleyMotorId) {
		//m_elevatorMotor = new SparkMax(motorId, MotorType.kBrushless);
		//m_elevatorMotorTwo = new SparkMax(motorTwoId, MotorType.kBrushless);
		//m_trolleyMotor = new SparkMax(trolleyMotorId, MotorType.kBrushless);
		m_elevatorMotor = new Motor(motorId, 20);
		m_elevatorMotor.setPid(ElevatorConstants.elevatorMotorP, ElevatorConstants.elevatorMotorI, ElevatorConstants.elevatorMotorD);
		m_elevatorMotor.setInverted(ElevatorConstants.elevatorMotorInverted);
		m_elevatorMotor.burnConfig();
		m_elevatorMotorController = m_elevatorMotor.getMotor().getClosedLoopController();

		m_elevatorMotorTwo = new Motor(motorTwoId, 20);
		m_elevatorMotorTwo.setPid(ElevatorConstants.elevatorMotorTwoP, ElevatorConstants.elevatorMotorTwoI, ElevatorConstants.elevatorMotorTwoD);
		m_elevatorMotorTwo.setInverted(ElevatorConstants.elevatorMotorTwoInverted);
		m_elevatorMotorTwo.burnConfig();
		m_elevatorMotorControllerTwo = m_elevatorMotorTwo.getMotor().getClosedLoopController();

		m_trolleyMotor = new Motor(trolleyMotorId, 20);
		m_trolleyMotor.setPid(ElevatorConstants.trolleyMotorP, ElevatorConstants.trolleyMotorI, ElevatorConstants.trolleyMotorD);
		m_trolleyMotor.setInverted(ElevatorConstants.trolleyMotorInverted);
		m_trolleyMotor.burnConfig();
		m_trolleyMotorController = m_trolleyMotor.getMotor().getClosedLoopController();

	}

	void setElevatorSpeed(double speedMult) {
		m_elevatorMotor.setSpeed(speedMult * ElevatorConstants.elevatorMaxSpeed);
		m_elevatorMotorTwo.setSpeed(speedMult * ElevatorConstants.elevatorMaxSpeed);
	}

	void setTrolleySpeed(double speedMult) {
		m_trolleyMotor.setSpeed(speedMult * ElevatorConstants.elevatorMaxSpeed);
	}

	void setElevatorReference(double position) {
		m_elevatorMotorController.setReference(position, ControlType.kPosition);
		m_elevatorMotorControllerTwo.setReference(position, ControlType.kPosition);
	}

	void setTrolleyReference(double position) {
		m_trolleyMotorController.setReference(position, ControlType.kPosition);
	}

}
