package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Robot;
import frc.robot.led.LEDControllerNew;
import frc.robot.led.LEDControllerNew.LEDMode;

public class DriveClimber extends CommandBase {

  private Climber climber = null;

  public DriveClimber() {
    climber = Climber.getInstance();

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.zeroLift();
    climber.fullStop();
  }

  @Override
  public void execute() {
    double liftOutput = Robot.operator.getControllerRTrigger();
    double winchOutput = liftOutput;

    climber.setShifterOutput(Robot.operator.getControllerRightStickX());

    if (Math.abs(liftOutput) > 0.25) {
      liftOutput = 0.25;
    }

    if (Robot.operator.getControllerBButton() == true) {
      LEDControllerNew.getInstance().setMode(LEDMode.WINCH_ACTIVE);

      if (winchOutput > 0) {
        if (climber.getLiftTicks() <= 0.50) {
          climber.setWinchOutput(0);
        }
        else {
          climber.setWinchOutput(winchOutput);
        }
      }
      else {
        climber.setWinchOutput(winchOutput);
      }
    }
    else {
      //LEDControllerNew.getInstance().setMode(LEDMode.CLIMB_ACTIVE);

      climber.setLiftOutput(liftOutput);
      climber.setWinchOutput(0);
    }

    if(climber.getLiftOutput() == 0 && climber.getWinchOutput() == 0) { 
      LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
