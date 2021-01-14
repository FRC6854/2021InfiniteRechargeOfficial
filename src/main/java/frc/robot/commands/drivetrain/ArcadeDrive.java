package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.led.LEDControllerNew;
import frc.robot.led.LEDControllerNew.LEDMode;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.KitDrivetrain;

import viking.controllers.PIDController;

import viking.Limelight;
import viking.Limelight.LightMode;

public class ArcadeDrive extends CommandBase {

  private Limelight limelight = null;
  private PIDController aimPIDController = null;
  private KitDrivetrain drivetrain = null;

  public ArcadeDrive() {
    limelight = Limelight.getInstance();
    drivetrain = KitDrivetrain.getInstance();
    aimPIDController = new PIDController(Constants.AIM_kP, Constants.AIM_kI, Constants.AIM_kD);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (Robot.driver.getControllerAButton() == true) {
      // Setup Limelight for targeting
      limelight.setLEDMode(LightMode.ON);
      limelight.setDriverMode(false);

      if (limelight.validTargets() == true) {
        SmartDashboard.putNumber("Limelight Y", limelight.targetY());

        // Calculate the PID output using a threshold and make the target 0 (center of the crosshair)
        double pidAim = aimPIDController.calcPID(0, -limelight.targetX(), Constants.AIM_kThreshold);

        if (Math.abs(pidAim) > Constants.AIM_kMaxCommand) {
          if      (pidAim > 0) pidAim = Constants.AIM_kMaxCommand;
          else if (pidAim < 0) pidAim = -Constants.AIM_kMaxCommand;
        }

        if (aimPIDController.isDone() == true) {
          LEDControllerNew.getInstance().setMode(LEDMode.VISION);
        }
        else {
          LEDControllerNew.getInstance().setMode(LEDMode.NO_VISION);
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

      LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);

      drivetrain.arcadeDrive(Robot.driver.getControllerLeftStickY(), Robot.driver.getControllerRightStickX());
    }
  }
}
