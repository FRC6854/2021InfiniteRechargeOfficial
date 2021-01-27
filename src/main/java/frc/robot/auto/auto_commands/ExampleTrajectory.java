// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.auto_commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.KitDrivetrain;

public class ExampleTrajectory extends SequentialCommandGroup {
  private KitDrivetrain drivetrain;

  public ExampleTrajectory() {
    drivetrain = KitDrivetrain.getInstance();

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      drivetrain.getTrajectoryConfig()
    );
    
    addCommands(drivetrain.createRamseteCommand(exampleTrajectory));
  }
}
