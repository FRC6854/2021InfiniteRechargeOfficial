package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.KitDrivetrain;

public class DriveAngle extends CommandBase implements Constants {
  private KitDrivetrain drivetrain = null;

  final double toleranceDegrees = 1.5;
  final int waitForTime = 15;
  
  double angle;
  double speed;
  boolean withinTolerance = false;

  int timer = 0;

  public DriveAngle(double angle, double speed) {
    drivetrain = KitDrivetrain.getInstance();

    addRequirements(drivetrain);

    this.angle = angle;
    this.speed = speed;

    drivetrain.changeGyroPID(GYRO_kP, GYRO_kI, GYRO_kD);
  }

  @Override
  public void execute() {
    drivetrain.turn(angle, speed, toleranceDegrees);
    withinTolerance = (drivetrain.getGyroAngle() < (angle + toleranceDegrees) && drivetrain.getGyroAngle() > (angle - toleranceDegrees));

    if (withinTolerance) {
      timer++;
    }
  }

  @Override
  public boolean isFinished() {
    if (withinTolerance && timer > waitForTime) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }
}
