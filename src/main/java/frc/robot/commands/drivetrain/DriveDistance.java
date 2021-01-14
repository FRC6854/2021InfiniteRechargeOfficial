package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KitDrivetrain;

public class DriveDistance extends CommandBase {
  private KitDrivetrain drivetrain = null;
  
  private double meters;

  private int timer = 0;

  final int waitForTime = 15;

  public DriveDistance(double meters) {
    drivetrain = KitDrivetrain.getInstance();

    addRequirements(drivetrain);

    this.meters = meters;
  }

  @Override
  public void initialize() {
    drivetrain.zeroSensors();
    drivetrain.driveMeters(meters);
  }

  @Override
  public void execute() {
    timer++;
  }

  @Override
  public boolean isFinished() {
    return (timer > waitForTime && (drivetrain.getLeftVelocity() == 0 && drivetrain.getRightVelocity() == 0));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }
}
