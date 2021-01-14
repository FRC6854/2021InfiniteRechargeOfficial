package frc.robot.auto.auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.led.LEDControllerNew;
import frc.robot.led.LEDControllerNew.LEDMode;
import frc.robot.subsystems.Conveyor;

public class RunConveyorTime extends CommandBase {

  private Conveyor conveyor = null;

  private double[][] points;
  private Timer timer;

  private int lastPoint = 0;

  /**
   * Run the conveyor at certain times at certain speeds
   * @param points [i][0] is the time, and [i][1] is the speed of the intake conveyor at that time, and [i][2] is the speed of the upper conveyor
   */
  public RunConveyorTime(double[][] points) {
    conveyor = Conveyor.getInstance();

    this.points = points;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.start();
    conveyor.fullStop();
  }

  @Override
  public void execute() {
    double lastOutputIntake = 0;
    double lastOutputUpper = 0;

    for (int i = 0; i < points.length; i++) {
      if (timer.get() >= points[i][0]) {
        lastPoint = i;
        lastOutputIntake = points[i][1];
        lastOutputUpper = points[i][2];
      }
    }

    // Check intake direction
    if (lastOutputIntake > 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.LCF);
    } else if (lastOutputIntake < 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.LCB);
    }

    // Check upper direction
    if (lastOutputUpper > 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.UCF);
    } else if (lastOutputUpper < 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.UCB);
    }

    // Check both directions
    if (lastOutputUpper > 0 && lastOutputIntake > 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.BOTH_FWRD);
    } else if (lastOutputUpper < 0 && lastOutputIntake < 0) {
      LEDControllerNew.getInstance().setMode(LEDMode.BOTH_BKWD);
    }

    conveyor.setOutputIntake(lastOutputIntake);
    conveyor.setOutputUpper(lastOutputUpper);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    conveyor.fullStop();
    LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);
  }

  @Override
  public boolean isFinished() {
    if (lastPoint == points.length) {
      return true;
    }
    return false;
  }
}
