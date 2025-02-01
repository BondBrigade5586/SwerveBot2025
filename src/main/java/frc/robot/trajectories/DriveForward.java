package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.trajectories.StartingPositions;

class DriveForward {

	private Trajectory m_startingPos, m_endingPos;

	DriveForward(int feetToDrive) {
		m_startingPos = TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),

		)
	}
};

