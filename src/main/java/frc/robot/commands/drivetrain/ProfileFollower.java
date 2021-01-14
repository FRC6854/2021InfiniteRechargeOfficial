package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.KitDrivetrain;
import viking.ProfileBuffer;

public class ProfileFollower extends CommandBase {

  private KitDrivetrain drivetrain = null;

  private ProfileBuffer buffer = null;

  public ProfileFollower(ProfileBuffer buffer) {
    drivetrain = KitDrivetrain.getInstance();

    this.buffer = buffer;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0);

    drivetrain.zeroSensors();
    drivetrain.resetMotionProfile();

    drivetrain.getLeftMaster().initMotionBuffer(buffer.getLeftProfile(), buffer.getLeftProfile().length);
    drivetrain.getRightMaster().initMotionBuffer(buffer.getRightProfile(), buffer.getRightProfile().length);

    drivetrain.motionProfile();
  }

  @Override
  public boolean isFinished() {
    return (drivetrain.getLeftMaster().isMotionProfileFinished() && drivetrain.getRightMaster().isMotionProfileFinished());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.resetMotionProfile();
  }
}
