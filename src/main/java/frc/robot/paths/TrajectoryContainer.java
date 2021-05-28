package frc.robot.paths;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * Interface for creating Trajectories in source
 * This makes creating Trajectories a little easier to understand
 */
public interface TrajectoryContainer {

	boolean isReversed();
	Trajectory buildTrajectory();
	String name();
}
