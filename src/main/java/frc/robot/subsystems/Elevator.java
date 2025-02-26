package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private Motor m_motor, m_motorTwo;
	private double m_countsPerRevolution = 0;

	public Elevator(int motorId, int motorTwoId) {
		
		m_motor = new Motor(motorId, 20, IdleMode.kBrake);
		m_motor.setPid(ElevatorConstants.elevatorMotorP, ElevatorConstants.elevatorMotorI, ElevatorConstants.elevatorMotorD);
		// m_motor.setMaxVelocity(ElevatorConstants.elevatorMaxSpeed);
		// m_motor.setMaxAcceleration(ElevatorConstants.elevatorMaxAcceleration);
		m_motor.setInverted(ElevatorConstants.elevatorMotorInverted);
		m_motor.burnConfig();
		
		m_motorTwo = new Motor(motorTwoId, 20,IdleMode.kBrake);
		m_motorTwo.copyConfig(motorId);
		m_motorTwo.burnConfig();
	}

	public double getPosition() {
		return m_motor.getEncoder().getPosition();
	}

	public Command moveElevatorCommand(double speedMult) {
		// return this.run(() -> setElevatorSpeed(speedMult));
		return this.run(() -> {
			m_motor.setSpeed(0.1);
			m_motorTwo.setSpeed(0.1);
		});
	}

	public void setElevatorSpeed(double speedMult) {
		// m_motor.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
		// m_motorTwo.setPIDVelocity(ElevatorConstants.elevatorMaxSpeed * speedMult);
		m_motor.setSpeed(speedMult);
		m_motorTwo.setSpeed(speedMult);
	}

	public void resetEncoders() {
		m_motor.getEncoder().setPosition(0.0);
		m_motorTwo.getEncoder().setPosition(0.0);
	}

	public void stop() {
		m_motor.stop();
		m_motorTwo.stop();
	}

	public Command stopElevatorCommand() {
		// return this.runOnce(() -> setElevatorSpeed(0));
		return this.run(() -> {
			m_motor.stop();
			m_motorTwo.stop();
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(
			"Elevator Integrated Encoder Value",
			m_motor.getEncoder().getPosition()
		);
	}

}
