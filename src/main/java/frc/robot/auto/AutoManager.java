package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.auto_commands.ExampleTrajectory;
import frc.robot.auto.auto_commands.ExampleTrajectoryPathWeaver;
import frc.robot.commands.debug.LimelightCalibration;

public class AutoManager {

    private static AutoManager instance = null;

    private static SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

    private AutoManager () {
      autoChooser.setDefaultOption("Example Trajectory", 1);
      autoChooser.addOption("Example Trajectory File", 2);
      autoChooser.addOption("Limelight Calibration", 0);
    }

    public SendableChooser<Integer> getAutoChooser() {
      return autoChooser;
    }

    public Command getAutoChooserCommand() {
      switch (autoChooser.getSelected()) {
        case 1:
          return new ExampleTrajectory();
        case 2:
          return new ExampleTrajectoryPathWeaver("output/Circle");
        case 0:
          return new LimelightCalibration();
      }

      return null;
    }

    public static AutoManager getInstance() {
      if (instance == null) {
          instance = new AutoManager();
      }
      return instance;
    }
}
