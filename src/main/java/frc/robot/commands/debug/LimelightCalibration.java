package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.CommandBase;
import viking.Limelight;
import viking.Limelight.LightMode;

/**
 * This is for when you arrive to competition to tune the vision pipeline.
 */
public class LimelightCalibration extends CommandBase {

  private Limelight limelight = null;

  public LimelightCalibration() {
    limelight = Limelight.getInstance();
  }

  @Override
  public void initialize() {
    limelight.setDriverMode(false);
    limelight.setLEDMode(LightMode.ON);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setDriverMode(true);
    limelight.setLEDMode(LightMode.OFF);
  }
}
