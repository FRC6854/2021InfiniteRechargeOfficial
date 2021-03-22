package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import viking.led.LEDController;
import viking.led.LEDController.LEDMode;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.KitDrivetrain;

import viking.controllers.PIDController;

import viking.Limelight;
import viking.Limelight.LightMode;

public class ArcadeDrive extends CommandBase {

  private Limelight limelight = Limelight.getInstance();
  private PIDController aimPIDController = new PIDController(Constants.AIM_kP, Constants.AIM_kI, Constants.AIM_kD);
  private KitDrivetrain drivetrain = KitDrivetrain.getInstance();

  public ArcadeDrive() {
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (Robot.driver.getControllerYButtonPressed()) {
      CommandScheduler.getInstance().schedule(
        drivetrain.createRamseteCommand(drivetrain.loadToTrench).andThen(drivetrain.createRamseteCommand(drivetrain.trenchToLoad))
        .withInterrupt(() -> (Robot.driver.getControllerLeftStickX() > 0 || Robot.driver.getControllerLeftStickY() > 0 || Robot.driver.getControllerRightStickX() > 0 || Robot.driver.getControllerRightStickY() > 0))
      );
    }
    if (Robot.driver.getControllerAButton()) {
      // Setup Limelight for targeting
      limelight.setLEDMode(LightMode.ON);
      limelight.setDriverMode(false);

      if (limelight.validTargets()) {
        SmartDashboard.putNumber("Limelight Y", limelight.targetY());

        // Calculate the PID output using a threshold and make the target 0 (center of the crosshair)
        double pidAim = aimPIDController.calcPID(0, -limelight.targetX(), Constants.AIM_kThreshold);

        if (Math.abs(pidAim) > Constants.AIM_kMaxCommand) {
          if      (pidAim > 0) pidAim = Constants.AIM_kMaxCommand;
          else if (pidAim < 0) pidAim = -Constants.AIM_kMaxCommand;
        }

        if (aimPIDController.isDone()) {
          LEDController.getInstance().setMode(LEDMode.VISION);
        }
        else {
          LEDController.getInstance().setMode(LEDMode.NO_VISION);
        }

        // Drive using the output values
        drivetrain.arcadeDrive(Robot.driver.getControllerLeftStickY(), pidAim);
      }
      else {
        drivetrain.arcadeDrive(Robot.driver.getControllerLeftStickY(), 0);
      }
    }
    else {
      limelight.setLEDMode(LightMode.OFF);
      limelight.setDriverMode(true);

      LEDController.getInstance().setMode(LEDMode.DEFAULT);

      drivetrain.arcadeDrive(Robot.driver.getControllerLeftStickY(), Robot.driver.getControllerRightStickX());
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) drivetrain.arcadeDrive(0, 0);
  }
}
