package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.KitDrivetrain;

public class ZeroGyro extends InstantCommand {

  private static KitDrivetrain drivetrain = null;

  public ZeroGyro() {
    drivetrain = KitDrivetrain.getInstance();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.resetGyro();
  }
}
