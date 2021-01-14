package frc.robot.auto.auto_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LEDControllerNew;
import frc.robot.led.LEDControllerNew.LEDMode;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.KitDrivetrain;
import frc.robot.subsystems.Shooter;
import viking.Limelight;
import viking.Limelight.LightMode;
import viking.controllers.PIDController;

public class AimShoot extends CommandBase {

  private PIDController aimPIDController = null;

  private Limelight limelight = null;
  private KitDrivetrain drivetrain = null;
  private Conveyor conveyor = null;
  private Shooter shooter = null;

  public AimShoot() {
    drivetrain = KitDrivetrain.getInstance();
    shooter = Shooter.getInstance();
    limelight = Limelight.getInstance();
    conveyor = Conveyor.getInstance();
    aimPIDController = new PIDController(Constants.AIM_kP, Constants.AIM_kI, Constants.AIM_kD);

    addRequirements(drivetrain, shooter, conveyor);
  }

  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0);
    limelight.setLEDMode(LightMode.ON);
    limelight.setDriverMode(false);
  }

  @Override
  public void execute() {
    if (limelight.validTargets() == true) {
      double pidAim = aimPIDController.calcPID(0, -limelight.targetX(), Constants.AIM_kThreshold);

      if (Math.abs(pidAim) > Constants.AIM_kMaxCommand) {
        if      (pidAim > 0) pidAim = Constants.AIM_kMaxCommand;
        else if (pidAim < 0) pidAim = -Constants.AIM_kMaxCommand;
      }

      drivetrain.arcadeDrive(0, pidAim);

      if (aimPIDController.isDone() == true) {
        conveyor.setOutputUpper(0.65);
        conveyor.setOutputIntake(0.65);

        if (Limelight.getInstance().targetY() < Constants.MAX_LIMELIGHT_DISTANCE) {
          shooter.setOutputTop(Constants.MAX_LIMELIGHT_SPEED);
          shooter.setOutputBottom(Constants.MAX_LIMELIGHT_SPEED);
        }
        else {
          shooter.setOutputTop(Constants.MIN_SPEED);
          shooter.setOutputBottom(Constants.MIN_SPEED);
        }

        LEDControllerNew.getInstance().setMode(LEDMode.VISION);
      }
      else {
        conveyor.fullStop();
        shooter.fullStop();

        LEDControllerNew.getInstance().setMode(LEDMode.NO_VISION);
      }
    }
    else {
      shooter.fullStop();
      conveyor.fullStop();
      drivetrain.arcadeDrive(0, 0);
      LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);
    }
  }

  @Override
  public void end(boolean interrupted) {
    LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);
    limelight.setDriverMode(true);
    limelight.setLEDMode(LightMode.OFF);
    shooter.fullStop();
    conveyor.fullStop();
    drivetrain.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
