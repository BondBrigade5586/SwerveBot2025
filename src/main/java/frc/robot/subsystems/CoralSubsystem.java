package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
	private DutyCycleEncoder m_pivotEncoder;
	private Motor m_driveMotor, m_pivotMotor;

	public CoralSubsystem(int wheelDriveMotorId, int pivotMotorId) {
		
		m_driveMotor = new Motor(wheelDriveMotorId, 35, IdleMode.kBrake);
		m_driveMotor.setPid(CoralConstants.wheelDriveP, CoralConstants.wheelDriveI, CoralConstants.wheelDriveP);
		m_driveMotor.setInverted(CoralConstants.wheelDriveInverted);
		m_driveMotor.burnConfig();

		m_pivotMotor = new Motor(pivotMotorId, 35, IdleMode.kBrake);
		m_pivotMotor.setPid(CoralConstants.angleMotorP, CoralConstants.angleMotorI, CoralConstants.angleMotorD);
		m_pivotMotor.setInverted(CoralConstants.angleMotorInverted);
		m_pivotMotor.burnConfig();
		m_pivotEncoder = new DutyCycleEncoder(9);
	}

	public double getPosition() {
		return m_pivotEncoder.get() * 360;
	}

	public void intake(double speedMult) {
		m_driveMotor.setSpeed(speedMult);
	}

	public Command intakeCommand(double speedMult) {
		return this.run(() -> intake(speedMult));
	}

	public boolean intakeMotorIsNull() {
		return m_driveMotor.getMotor().getFaults().can;
	}

	public void outtake(double speedMult) {
		m_driveMotor.setSpeed(speedMult * -1);
	}

	public Command outtakeCommand(double speedMult) {
		return this.run(() -> outtake(speedMult));
	}

	public boolean pivotMotorIsNull() {
		return m_pivotMotor.getMotor().getFaults().can;
	}

	public void setArmSpeed(double speedMult) {
		m_pivotMotor.setSpeed(speedMult * 0.5);
	}

	public void stop() {
		m_driveMotor.setSpeed(0);
	}

	public void setArmVoltage(double voltage) {
		m_pivotMotor.setVoltage(voltage);
	}

	public void stopArm() {
		m_pivotMotor.setPidPosition(m_pivotMotor.getEncoder().getPosition());
	}

	public Command stopCommand() {
		return this.runOnce(() -> stop());
	}

	public Command stopArmCommand() {
		return this.runOnce(() -> m_pivotMotor.setSpeed(0));
	}

	public BooleanSupplier withinBounds() {
		double position = this.getPosition();
		return () -> (position > CoralConstants.minPos && position < CoralConstants.maxPos);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Coral Encoder Value", getPosition());
	}
	
}
