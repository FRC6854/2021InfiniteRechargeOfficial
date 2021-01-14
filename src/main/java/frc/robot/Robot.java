package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoManager;
import frc.robot.auto.auto_commands.AimShoot;
import frc.robot.led.LEDControllerNew;
import frc.robot.led.LEDControllerNew.LEDMode;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.KitDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import viking.Controller;
import viking.Limelight;
import viking.OI;
import viking.Limelight.LightMode;

public class Robot extends TimedRobot implements RobotMap {

  public static Controller driver = null;
  public static Controller operator = null;

  private static AutoManager autoManager = null;

  private static UsbCamera camera = null;

  @Override
  public void robotInit() {
    camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setFPS(30);

    Limelight.getInstance().setLEDMode(LightMode.OFF);
    Limelight.getInstance().setDriverMode(true);

    autoManager = AutoManager.getInstance();

    driver = new Controller(CONTROLLER_DRIVER);

    operator = new Controller(CONTROLLER_OPERATOR);
    operator.setControllerRightStickXDeadband(0.05);
    
    KitDrivetrain.getInstance();
    Conveyor.getInstance();
    Shooter.getInstance();
    Climber.getInstance();

    OI.getInstance();
  }

  @Override
  public void robotPeriodic() {
    // Data display stuff
    SmartDashboard.putData(autoManager.getAutoChooser());

    // Low Voltage Display
    if (RobotController.getBatteryVoltage() < 10) {
      LEDControllerNew.getInstance().setMode(LEDMode.LOW_VOLTAGE);
    }
  }

  @Override
  public void disabledInit() {
    System.out.println("Disabled");
    LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous");

    LEDControllerNew.getInstance().setMode(LEDMode.DEFAULT);

    Command autoCommand = autoManager.getAutoChooserCommand();
    if (autoCommand != null) {
      CommandScheduler.getInstance().schedule(autoCommand);
    }
    else {
      CommandScheduler.getInstance().schedule(new AimShoot().withTimeout(15));
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    System.out.println("Tele-op");
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

}
