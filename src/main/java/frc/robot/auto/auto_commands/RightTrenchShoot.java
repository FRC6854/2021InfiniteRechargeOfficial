package frc.robot.auto.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveAngle;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.ProfileFollower;
import frc.robot.commands.drivetrain.ZeroGyro;
import viking.ProfileBuffer;

public class RightTrenchShoot extends SequentialCommandGroup {

  private static ProfileBuffer ballShoot = new ProfileBuffer("ball-shoot");

  public RightTrenchShoot() {
    super(
      new ZeroGyro(),
      new DriveAngle(-20, 0.5).withTimeout(1.0),
      new AimShoot().withTimeout(3.0),
      new DriveAngle(0, 0.5).withTimeout(1.0),
      new DriveDistance(5).raceWith(
        new RunConveyorTime(
          new double[][] {
            // After 1 second, drive both conveyors at 50% speed, then after 5 seconds stop
            {1.0, 0.5, 0.5},
            {5.0, 0.0, 0.0}
          }
        )
      ),
      new ProfileFollower(ballShoot), 
      new AimShoot().withTimeout(4)
    );
  }
}
