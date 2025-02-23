package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;

public class AutoTrajectory {

	private Trajectory m_trajectory;
	private Pose2d m_startingPos;
	private ArrayList<Pose2d> m_poseList;
	private TrajectoryConfig m_trajectoryConfig;
	private PIDController m_xController;
	private PIDController m_yController;
	private double m_xPos = 0, m_yPos = 0;

	private ProfiledPIDController m_thetaController;

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

	public void generateTrajectory(ArrayList<Pose2d> list) {
		m_trajectory = TrajectoryGenerator.generateTrajectory(list, m_trajectoryConfig);
	}

	public PIDController getYController() {
		return m_yController;
	}

	public PIDController getXController() {
		return m_xController;
	}

	public Pose2d getPoseAtIndex(int i) {
		return m_poseList.get(i);
	}

	public ArrayList<Pose2d> getPositions() {
		return m_poseList;
	}

	public ProfiledPIDController getThetaController() {
		return m_thetaController;
	}

	public Trajectory getTrajectory() {
		return m_trajectory;
	}

	public double getTrajectoryDuratrion() {
		return m_trajectory.getTotalTimeSeconds();
	}

	public void modifyStartPosition(Pose2d pose) {
		m_poseList.set(0, pose);
	}

	public void pushPosition(Pose2d pose) {
		m_poseList.add(pose);
	}

	public void pushPosition(double x, double y, double rotationDegrees) {
		Pose2d m_pos = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
		m_poseList.add(m_pos);
	}

	public void translate(double x, double y) {
		Transform2d transform = new Transform2d(x, y, Rotation2d.fromDegrees(0.0));
		m_poseList.add(m_poseList.get(m_poseList.size() - 1).plus(transform));
	}

	public void transform(Transform2d transform) {
		Pose2d tempPose = m_poseList.get(m_poseList.size() - 1);
		tempPose.transformBy(transform);
		m_poseList.add(tempPose);
	}

	private void configureControllers() {
		m_xController = new PIDController(AutoConstants.autoXController, 0.0, 0.0);
		m_yController = new PIDController(AutoConstants.autoYController, 0.0, 0.0);

		m_thetaController = new ProfiledPIDController(
			AutoConstants.autoThetaControllerVal,
			0.0,
			0.0,
			AutoConstants.autoThetaControllerConstraints
		);
		m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}
};

