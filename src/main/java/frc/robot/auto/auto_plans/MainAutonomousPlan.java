// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.auto_plans;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.auto_commands.AutoDriveTrajectory;
import frc.robot.auto.auto_commands.RunConveyorTime;
import frc.robot.subsystems.Constants;

public class MainAutonomousPlan extends SequentialCommandGroup {

  public MainAutonomousPlan() {
    ArrayList<Spline.ControlVector> lineToTrench = new ArrayList<Spline.ControlVector>();
    lineToTrench.add(new Spline.ControlVector(new double[]{3.1, 1.285, 0}, new double[]{-2.4, 0, 0}));
    lineToTrench.add(new Spline.ControlVector(new double[]{4.15, 1.011, 0}, new double[]{-1.55, 0, 0}));
    lineToTrench.add(new Spline.ControlVector(new double[]{6.5, 2.879, 0}, new double[]{-1.55, 0, 0}));

    Trajectory lineToTrenchTrajectory = TrajectoryGenerator.generateTrajectory(
      new TrajectoryGenerator.ControlVectorList(lineToTrench),
      Constants.DRIVETRAIN_kAutoConfig
    );

    ArrayList<Spline.ControlVector> trenchToLine = new ArrayList<Spline.ControlVector>();
    trenchToLine.add(new Spline.ControlVector(new double[]{6.5, 2.879, 0}, new double[]{-1.55, 0, 0}));
    trenchToLine.add(new Spline.ControlVector(new double[]{4.15, 1.011, 0}, new double[]{-1.55, 0, 0}));
    trenchToLine.add(new Spline.ControlVector(new double[]{3.1, 1.285, 0}, new double[]{-2.4, 0, 0}));
 
    Trajectory trenchToLineTrajectory = TrajectoryGenerator.generateTrajectory(
      new TrajectoryGenerator.ControlVectorList(trenchToLine),
      Constants.DRIVETRAIN_kAutoConfig.setReversed(true)
    );

    addCommands(
      new ParallelCommandGroup(
        new AutoDriveTrajectory(lineToTrenchTrajectory, true),
        new RunConveyorTime(new double[][] {
          {3, 1, 0.1},
          {6, 0, 0}
        })),
      new AutoDriveTrajectory(trenchToLineTrajectory, true)
    );
  }
}
