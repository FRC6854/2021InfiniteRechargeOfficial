package frc.robot.subsystems;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public interface Constants {
	
	/**
	 * --------------------
	 * 	DRIVETRAIN CONSTANTS
	 * --------------------
	 */
    public final double DRIVETRAIN_kP = 1.0;
    public final double DRIVETRAIN_kI = 0.0; 
    public final double DRIVETRAIN_kD = 0.0;
	public final double DRIVETRAIN_kF = 0.5;
	
	public final double DRIVETRAIN_kMetersPerRevolution = 2 * Math.PI * 0.0762;
	public final double DRIVETRAIN_kWheelRadius = 0.0762;

	public final double DRIVETRAIN_kDriveGearing = 10.75;

	public final int DRIVETRAIN_kCPR = 4096;

	public final DifferentialDriveKinematics DRIVETRAIN_kKinematics = new DifferentialDriveKinematics(0.9);

	public final double DRIVETRAIN_kMaxSpeed = 2;
	public final double DRIVETRAIN_kMaxAcceleration = 3;
	public final double DRIVETRAIN_kRamseteB = 2;
	public final double DRIVETRAIN_kRamseteZeta = 0.7;
	
	// ----------------------------------------------
	// PATH FOLLOWING VARIABLES FOR TESTING MUST USE PROGRAM TO FIND ACTUAL VALUES
	// ----------------------------------------------
	// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public final double DRIVETRAIN_ksVolts = 0.22;
    public final double DRIVETRAIN_kvVoltSecondsPerMeter = 1.98;
    public final double DRIVETRAIN_kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public final double DRIVETRAIN_kPVelocity = 1;

	/**
	 * --------------------
	 * 	GYRO CONSTANTS
	 * --------------------
	 */
	public final double GYRO_kP = 0.025;
	public final double GYRO_kI = 0.0;
	public final double GYRO_kD = 0.2;

	/**
	 * -------------------------
	 * 	VISION AIMING CONSTANTS
	 * -------------------------
	 */
	public final double AIM_kP = 0.05;
	public final double AIM_kI = 0.00003;
	public final double AIM_kD = 0.02;

	public final double AIM_kMaxCommand = 0.75;
	public final double AIM_kThreshold = 3;

	/**
	 * --------------------
	 *  CONVEYOR CONSTANTS
	 * --------------------
	 */
	public final double CONVEYOR_kP = 0;
	public final double CONVEYOR_kI = 0;
	public final double CONVEYOR_kD = 0;
	public final double CONVEYOR_kF = 0;
	public final double CONVEYOR_ACCELERATION = 0;
	public final double CONVEYOR_MAX_VELOCITY = 0;

	/**
	 * --------------------
	 *   SHOOTER CONSTANTS
	 * --------------------
	 */

	
	public final double SHOOTER_kP = 0.001;
	public final double SHOOTER_kI = 0;
	public final double SHOOTER_kD = 0.005;
	public final double SHOOTER_kF = 0;
	public final double SHOOTER_ACCELERATION = 0;
	public final double SHOOTER_MAX_VELOCITY = 0;

	public final double MAX_LIMELIGHT_DISTANCE = -7;
	public final double MAX_LIMELIGHT_SPEED = 0.60;

	public final double MIN_SPEED = 0.55;

	/**
	 * --------------------
	 *   CLIMBER CONSTANTS
	 * --------------------
	 */
	public final double LIFT_kP = 0;
	public final double LIFT_kI = 0;
	public final double LIFT_kD = 0;
	public final double LIFT_kF = 0;
	public final double LIFT_ACCELERATION = 0;
	public final double LIFT_MAX_VELOCITY = 0;
	public final float LIFT_MAX_ROTATIONS = 19f;

	public final double WINCH_kP = 0;
	public final double WINCH_kI = 0;
	public final double WINCH_kD = 0;
	public final double WINCH_kF = 0;
	public final double WINCH_ACCELERATION = 0;
	public final double WINCH_MAX_VELOCITY = 0;

	public final double SHIFT_kP = 0;
	public final double SHIFT_kI = 0;
	public final double SHIFT_kD = 0;
	public final double SHIFT_kF = 0;
	public final double SHIFT_ACCELERATION = 0;
	public final double SHIFT_MAX_VELOCITY = 0;
}
