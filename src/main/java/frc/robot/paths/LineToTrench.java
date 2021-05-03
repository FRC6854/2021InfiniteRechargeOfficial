package frc.robot.paths;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Constants;

public class LineToTrench implements TrajectoryContainer {

    @Override
    public boolean isReversed() {
        return false;
    }

    @Override
    public Trajectory buildTrajectory() {
        ArrayList<Spline.ControlVector> trajectory = new ArrayList<Spline.ControlVector>();
        
        // Vector values with [x,y] last value isn't important unless you know what you are doing
        trajectory.add(new Spline.ControlVector(new double[]{3.1, 1.285, 0}, new double[]{2.4, 0, 0}));
        trajectory.add(new Spline.ControlVector(new double[]{4.15, 1.011, 0}, new double[]{3.95, 0, 0}));
        trajectory.add(new Spline.ControlVector(new double[]{6.5, 2.879, 0}, new double[]{3.95, 0, 0}));
        
        return TrajectoryGenerator.generateTrajectory(
            new TrajectoryGenerator.ControlVectorList(trajectory),
            Constants.DRIVETRAIN_kAutoConfig.setReversed(isReversed())
        );
    }

    @Override
    public String name() {
        return "Line To Trench";
    }
}
