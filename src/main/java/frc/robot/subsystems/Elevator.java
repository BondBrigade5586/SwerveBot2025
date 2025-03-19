package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	private Motor m_motor, m_motorTwo;
	private Rev2mDistanceSensor m_distanceSensor;
	private LaserCan m_laserCan;

	public Elevator(int motorId, int motorTwoId) {
		
		m_motor = new Motor(motorId, 33, IdleMode.kCoast);
		m_motor.setPid(ElevatorConstants.elevatorMotorP, ElevatorConstants.elevatorMotorI, ElevatorConstants.elevatorMotorD);
		// m_motor.setMaxVelocity(ElevatorConstants.elevatorMaxSpeed);
		// m_motor.setMaxAcceleration(ElevatorConstants.elevatorMaxAcceleration);
		m_motor.setInverted(ElevatorConstants.elevatorMotorInverted);
		m_motor.burnConfig();
		
		m_motorTwo = new Motor(motorTwoId, 33,IdleMode.kCoast);
		m_motor.setInverted(ElevatorConstants.elevatorMotorTwoInverted);
		m_motorTwo.copyConfig(motorId);
		m_motorTwo.burnConfig();

		// m_distanceSensor = new Rev2mDistanceSensor(Port.kMXP);
		// m_distanceSensor.setAutomaticMode(true);
		// m_distanceSensor.setRangeProfile(RangeProfile.kLongRange);

		m_laserCan = new LaserCan(ElevatorConstants.distanceSensorId);
		CanBridge.runTCP();
	}

	public double getMotorOneAmperage() {
		return m_motor.getMotor().getOutputCurrent();
	}

	public double getMotorTwoAmperage() {
		return m_motorTwo.getMotor().getOutputCurrent();
	}
	
	public double getElevatorDistance() {
		if (m_laserCan.getMeasurement() == null) return -1;
		return m_laserCan.getMeasurement().distance_mm / 25.4;
		// return m_distanceSensor.getRange(Unit.kInches);
	}

	public boolean motorsAreNull() {
		return (m_motor.getMotor().getFaults().can || m_motorTwo.getMotor().getFaults().can);
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

	public void setElevatorVoltage(double voltage) {
		m_motor.setVoltage(voltage);
		m_motorTwo.setVoltage(voltage);
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

	public BooleanSupplier withinBounds() {
		return (() -> {
			return (getElevatorDistance() > ElevatorConstants.minDistance && getElevatorDistance() < ElevatorConstants.maxDistance);
		});
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(
			"Elevator Integrated Encoder Value",
			m_motor.getEncoder().getPosition()
		);

		// SmartDashboard.putBoolean("Elevator/Distance Sensor Status", m_distanceSensor.isEnabled());
		SmartDashboard.putNumber("Elevator/Distance Sensor Range", getElevatorDistance());
		// SmartDashboard.putBoolean("Elevator/Valid Distance Sensor Range", m_distanceSensor.isRangeValid());
		SmartDashboard.putNumber("Eelvator/Motor One Current", getMotorOneAmperage());
		SmartDashboard.putNumber("Eelvator/Motor Two Current", getMotorTwoAmperage());
	}

}
