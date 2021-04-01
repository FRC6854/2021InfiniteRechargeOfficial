package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.auto_commands.AutoDriveTrajectory;
import frc.robot.auto.auto_commands.RunConveyorTime;
import frc.robot.commands.debug.LimelightCalibration;
import frc.robot.paths.LineToTrench;
import frc.robot.paths.TrenchToLine;

public class AutoManager {

    private static AutoManager instance = null;

    private static SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();

    private AutoManager () {
      autoChooser.setDefaultOption("Main Autonomous", 1);
      autoChooser.addOption("Limelight Calibration", 0);
    }

    public SendableChooser<Integer> getAutoChooser() {
      return autoChooser;
    }

    public Command getAutoChooserCommand() {
      switch (autoChooser.getSelected()) {
        case 1:
          return new SequentialCommandGroup(
            new ParallelCommandGroup(
              new AutoDriveTrajectory(new LineToTrench(), true),
              new RunConveyorTime(new double[][] {
                {3, 1, 0.1},
                {6, 0, 0}
              })),
            new AutoDriveTrajectory(new TrenchToLine(), true)
          );
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
