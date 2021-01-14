package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Shooter;
import viking.Limelight;

public class DriveShooter extends CommandBase {

  private Shooter shooter = null;
  
  public DriveShooter() {
    shooter = Shooter.getInstance();
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (Robot.driver.getControllerRBumper() == true) {
      if (Limelight.getInstance().validTargets() == true) {
        if (Limelight.getInstance().targetY() < Constants.MAX_LIMELIGHT_DISTANCE) {
          shooter.setOutputTop(Constants.MAX_LIMELIGHT_SPEED);
          shooter.setOutputBottom(Constants.MAX_LIMELIGHT_SPEED);
        }
        else {
          shooter.setOutputTop(Constants.MIN_SPEED);
          shooter.setOutputBottom(Constants.MIN_SPEED);
        }
      }
      else {
        shooter.setOutputTop(Constants.MIN_SPEED);
        shooter.setOutputBottom(Constants.MIN_SPEED);
      }
      
    }
    else if (Robot.driver.getControllerLBumper() == true) {
      shooter.setOutputTop(0.3);
      shooter.setOutputBottom(-0.35);
    }
    else {
      shooter.fullStopTop();
      shooter.fullStopBottom();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.fullStopTop();
    shooter.fullStopBottom();
  }
}
