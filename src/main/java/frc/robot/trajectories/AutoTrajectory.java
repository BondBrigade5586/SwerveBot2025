package frc.robot.trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;

public class AutoTrajectory {

	Trajectory m_trajectory;
	Pose2d m_startingPos;
	ArrayList<Pose2d> m_poseList;
	TrajectoryConfig m_trajectoryConfig;
	PIDController m_xController;
	PIDController m_yController;

	ProfiledPIDController m_thetaController;

	public AutoTrajectory (double maxSpeed, double maxAcceleration) {

		m_poseList = new ArrayList<Pose2d>();
		m_startingPos = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
		m_poseList.add(m_startingPos);

		m_trajectoryConfig = new TrajectoryConfig(Units.feetToMeters(maxSpeed), Units.feetToMeters(maxAcceleration));

		configureControllers();

	}

	public void generateTrajectory() {
		m_trajectory = TrajectoryGenerator.generateTrajectory(m_poseList, m_trajectoryConfig);
	}

	public void pushPosition(double x, double y, double rotationDegrees) {
		Pose2d m_pos = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
		m_poseList.add(m_pos);
	}

	public void pushPosition(Pose2d pose) {
		m_poseList.add(pose);

	}

	public void modifyStartPosition(Pose2d pose) {
		m_poseList.set(0, pose);
	}

	public ArrayList<Pose2d> getPositions() {
		return m_poseList;
	}

	public PIDController getController(char axis) {
		return (Character.toLowerCase(axis) == 'y') ? m_yController : m_xController;
	}

	public ProfiledPIDController getThetaController() {
		return m_thetaController;
	}

	public Pose2d getPoseAtIndex(int i) {
		return m_poseList.get(i);
	}

	public double getTrajectoryDuratrion() {
		return m_trajectory.getTotalTimeSeconds();
	}

	public Trajectory getTrajectory() {
		return m_trajectory;
	}

	private void configureControllers() {
		m_xController = new PIDController(AutoConstants.autoXController, 0.0, 0.0);
		m_yController = new PIDController(AutoConstants.autoYController, 0.0, 0.0);

		m_thetaController = new ProfiledPIDController(
			AutoConstants.autoThetaController,
			0.0,
			0.0,
			AutoConstants.autoThetaControllerConstraints
		);
		m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}
};

