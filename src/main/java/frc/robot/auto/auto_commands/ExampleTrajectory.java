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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.KitDrivetrain;

public class ExampleTrajectory extends ParallelCommandGroup {
  private KitDrivetrain drivetrain;

  public ExampleTrajectory() {
    drivetrain = KitDrivetrain.getInstance();

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(2, 1)
      ),
      new Pose2d(4.2, 1, new Rotation2d(0)),
      Constants.DRIVETRAIN_kAutoConfig
    );
    
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> {
          drivetrain.resetGyro();
          drivetrain.resetOdemetry(exampleTrajectory.getInitialPose());
        }, drivetrain), 
        drivetrain.createRamseteCommand(exampleTrajectory)
      ),
      new RunConveyorTime(new double[][] {
        {3, 1, 0},
        {5, 0, 0}
      })
    );
  }
}
