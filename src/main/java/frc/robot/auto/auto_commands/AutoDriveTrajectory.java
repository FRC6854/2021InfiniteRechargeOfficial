package frc.robot.auto.auto_commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.paths.TrajectoryContainer;
import frc.robot.subsystems.KitDrivetrain;

public class AutoDriveTrajectory extends SequentialCommandGroup {
  private KitDrivetrain drivetrain;

  public AutoDriveTrajectory(TrajectoryContainer trajectory, boolean zeroGyro) {
    drivetrain = KitDrivetrain.getInstance();

    Trajectory traj = trajectory.buildTrajectory();
    Pose2d initialPose = traj.getInitialPose();
    RamseteCommand pathCommand = drivetrain.createRamseteCommand(traj);

    addCommands(
      new InstantCommand(() -> {
        if (zeroGyro) drivetrain.zeroSensors();
        drivetrain.resetOdemetry(initialPose);
      }, drivetrain),
      pathCommand,
      new InstantCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain)
    );
  }
}
