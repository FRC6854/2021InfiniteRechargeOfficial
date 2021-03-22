package frc.robot.paths;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

/** Interface for creating Trajectories in source */
public interface TrajectoryContainer {

    boolean isReversed();
    Trajectory buildTrajectory();
    String name();
}
