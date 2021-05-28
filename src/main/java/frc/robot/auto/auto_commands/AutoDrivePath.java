// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.auto_commands;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.KitDrivetrain;

public class AutoDrivePath extends SequentialCommandGroup {
	private KitDrivetrain drivetrain;

	public AutoDrivePath(String path, boolean zeroGyro) {
		drivetrain = KitDrivetrain.getInstance();

		Trajectory trajectory = new Trajectory();

		try {
			System.out.println("Loading trajectory...");
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path + ".wpilib.json");
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			System.out.println("Done loading trajectory!");
			Pose2d initialPose = trajectory.getInitialPose();

			RamseteCommand pathCommand = drivetrain.createRamseteCommand(trajectory);

			addCommands(
				new InstantCommand(() -> {
					if (zeroGyro) drivetrain.zeroSensors();
					drivetrain.resetOdemetry(initialPose);
				}, drivetrain),
				pathCommand,
				new InstantCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain)
			);

		} catch (Exception ex) {
			DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
		}
	}
}
