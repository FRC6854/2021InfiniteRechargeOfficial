package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoManager;
import frc.robot.auto.auto_commands.AimShoot;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.LEDController.LEDMode;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.KitDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import viking.Controller;
import viking.OI;
import viking.vision.Limelight;
import viking.vision.Limelight.LightMode;

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
		driver.setControllerLeftStickXDeadband(0.1);
		driver.setControllerLeftStickYDeadband(0.1);
		driver.setControllerRightStickXDeadband(0.1);
		driver.setControllerRightStickYDeadband(0.1);

		operator = new Controller(CONTROLLER_OPERATOR);
		operator.setControllerRightStickXDeadband(0.05);

		SmartDashboard.putData(KitDrivetrain.getInstance());
		SmartDashboard.putData(Conveyor.getInstance());
		Shooter.getInstance();
		Climber.getInstance();

		OI.getInstance();

		LiveWindow.disableAllTelemetry();
	}

	@Override
	public void robotPeriodic() {
		// Data display stuff
		SmartDashboard.putData(autoManager.getAutoChooser());

		// Low Voltage Display
		if (RobotController.getBatteryVoltage() < 10) {
			LEDController.getInstance().setMode(LEDMode.LOW_VOLTAGE);
		}

		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		System.out.println("Disabled");
		LEDController.getInstance().setMode(LEDMode.DEFAULT);
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void autonomousInit() {
		System.out.println("Autonomous");

		LEDController.getInstance().setMode(LEDMode.DEFAULT);

		Command autoCommand = autoManager.getAutoChooserCommand();
		if (autoCommand != null) {
			CommandScheduler.getInstance().schedule(autoCommand);
		}
		else {
			CommandScheduler.getInstance().schedule(new AimShoot().withTimeout(15));
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("Tele-op");
	}
}
