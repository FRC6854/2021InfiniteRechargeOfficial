// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.auto_commands;

import viking.motion.MotionProfileUtil;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.KitDrivetrain;

public class ExampleTrajectoryPathWeaver extends SequentialCommandGroup {
  private KitDrivetrain drivetrain;

  public ExampleTrajectoryPathWeaver(String... args) {
    drivetrain = KitDrivetrain.getInstance();

    Trajectory trajectory = new Trajectory();

    for (String name : args) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(name + ".wpilib.json");
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        Pose2d initialPose = trajectory.getInitialPose();

        addCommands(new InstantCommand(() -> drivetrain.resetOdemetry(initialPose), drivetrain), drivetrain.createRamseteCommand(trajectory));

      } catch (Exception ex) {
        DriverStation.reportError("Unable to open trajectory: " + name, ex.getStackTrace());
      }
    }
  }
}
