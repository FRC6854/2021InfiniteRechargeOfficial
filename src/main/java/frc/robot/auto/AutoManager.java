package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.auto_commands.MiddleTrenchShoot;
import frc.robot.auto.auto_commands.RightTrenchShoot;
import frc.robot.commands.debug.LimelightCalibration;

public class AutoManager {

    private static AutoManager instance = null;

    private static SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

    private AutoManager () {
      autoChooser.setDefaultOption("Middle Trench Shoot", 1);
      autoChooser.addOption("Right Trench Shoot", 2);
      autoChooser.addOption("Limelight Calibration", 0);
    }

    public SendableChooser<Integer> getAutoChooser() {
      return autoChooser;
    }

    public Command getAutoChooserCommand() {
      switch (autoChooser.getSelected()) {
        case 1:
          return new MiddleTrenchShoot();
        case 2:
          return new RightTrenchShoot();
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
