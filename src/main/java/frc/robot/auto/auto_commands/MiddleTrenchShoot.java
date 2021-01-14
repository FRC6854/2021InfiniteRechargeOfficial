package frc.robot.auto.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.ProfileFollower;
import viking.ProfileBuffer;

public class MiddleTrenchShoot extends SequentialCommandGroup {
  
  private static ProfileBuffer lineTrench = new ProfileBuffer("line-trench");
  private static ProfileBuffer ballShoot = new ProfileBuffer("ball-shoot");

  public MiddleTrenchShoot() {
    super(
      new AimShoot().withTimeout(3), 
      new ProfileFollower(lineTrench).raceWith(
        new RunConveyorTime(
          new double[][] {
            // After 1 second, drive both conveyors at 50% speed, then after 5 seconds stop
            {1.0, 1, 0.1},
            {6.5, 0.0, 0.0}
          }
        )
      ), 
      new ProfileFollower(ballShoot).withTimeout(3), 
      new AimShoot().withTimeout(3)
    );
  }
}
