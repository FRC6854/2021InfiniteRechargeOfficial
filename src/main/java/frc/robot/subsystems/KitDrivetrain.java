package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.kauailabs.navx.frc.AHRS;

import viking.controllers.ctre.VikingSPX;
import viking.controllers.ctre.VikingSRX;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.utils.UnitConversion;

public class KitDrivetrain extends SubsystemBase implements Constants, RobotMap {

  private static KitDrivetrain instance = null;

  private VikingSRX leftMaster;
  private VikingSPX leftSlave;

  private VikingSRX rightMaster;
  private VikingSPX rightSlave;

  private AHRS gyro;

  private PIDController gyroPID;

  private DifferentialDrive drive;
  private DifferentialDriveOdometry odometry;
  private DifferentialDrivetrainSim driveSim;
  private Field2d field;

  private TalonSRXSimCollection leftSim;
  private TalonSRXSimCollection rightSim;

  public Trajectory loadToTrench;
  public Trajectory trenchToLoad;

  public KitDrivetrain() {
    if (RobotBase.isReal()) {
      leftMaster = new VikingSRX(CAN_LEFT_FRONT, false, true, FeedbackDevice.CTRE_MagEncoder_Relative, DRIVETRAIN_kF, DRIVETRAIN_kP, DRIVETRAIN_kI, DRIVETRAIN_kD, 1250, 1250);
      leftSlave = new VikingSPX(CAN_LEFT_BACK, leftMaster, false);

      rightMaster = new VikingSRX(CAN_RIGHT_FRONT, false, false, FeedbackDevice.CTRE_MagEncoder_Relative, DRIVETRAIN_kF, DRIVETRAIN_kP, DRIVETRAIN_kI, DRIVETRAIN_kD, 1250, 1250);
      rightSlave = new VikingSPX(CAN_RIGHT_BACK, rightMaster, false);
    }
    else {
      leftMaster = new VikingSRX(CAN_LEFT_FRONT, false, false, FeedbackDevice.CTRE_MagEncoder_Relative, DRIVETRAIN_kF, DRIVETRAIN_kP, DRIVETRAIN_kI, DRIVETRAIN_kD, 1250, 1250);
      leftSlave = new VikingSPX(CAN_LEFT_BACK, leftMaster, false);

      rightMaster = new VikingSRX(CAN_RIGHT_FRONT, true, true, FeedbackDevice.CTRE_MagEncoder_Relative, DRIVETRAIN_kF, DRIVETRAIN_kP, DRIVETRAIN_kI, DRIVETRAIN_kD, 1250, 1250);
      rightSlave = new VikingSPX(CAN_RIGHT_BACK, rightMaster, true);
    }
    

    try {
      gyro = new AHRS(Port.kMXP);
    } catch (RuntimeException ex ) {
      System.out.println("--------------");
      System.out.println("NavX not plugged in");
      System.out.println("--------------");
    }

    gyroPID = new PIDController(GYRO_kP, GYRO_kI, GYRO_kD);
    drive = new DifferentialDrive(leftMaster, rightMaster);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
    field = new Field2d();

    SmartDashboard.putData("Field", field);

    if (RobotBase.isSimulation()) {
      driveSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2), // Number of motors per side
        10.75, // Simulation Gearing Ratio
        7.5,
        60, // Mass of robot
        DRIVETRAIN_kWheelRadius, 
        DRIVETRAIN_kKinematics.trackWidthMeters, 
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      );
     
      leftSim = leftMaster.getSimCollection();
      rightSim = rightMaster.getSimCollection();
    }

    reset();
  }

  @Override
  public void periodic() {
    odometry.update(
      Rotation2d.fromDegrees(getGyroAngle()), 
      UnitConversion.nativeUnitsToDistanceMeters(leftMaster.getTicks()),
      UnitConversion.nativeUnitsToDistanceMeters(rightMaster.getTicks()));
      
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    driveSim.setInputs(leftMaster.getMotorOutputVoltage() * (leftMaster.getInverted() ? -1 : 1), rightMaster.getMotorOutputVoltage() * (rightMaster.getInverted() ? -1 : 1));

    driveSim.update(0.02);

    leftSim.setQuadratureRawPosition(UnitConversion.distanceToNativeUnits(driveSim.getLeftPositionMeters()));
    leftSim.setQuadratureVelocity(UnitConversion.velocityToNativeUnits(driveSim.getLeftVelocityMetersPerSecond()));
    rightSim.setQuadratureRawPosition(UnitConversion.distanceToNativeUnits(driveSim.getRightPositionMeters()));
    rightSim.setQuadratureVelocity(UnitConversion.velocityToNativeUnits(driveSim.getRightVelocityMetersPerSecond()));

    // Crazy dumb NavX simulation stuff that I don't get
    int testing = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(testing, "Yaw"));
    angle.set(driveSim.getHeading().getDegrees());
  }

  public VikingSRX getLeftMaster() {
    return leftMaster;
  }

  public VikingSRX getRightMaster() {
    return rightMaster;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      UnitConversion.nativeUnitsToVelocityMetersPerSecond(leftMaster.getVelocity()), 
      UnitConversion.nativeUnitsToVelocityMetersPerSecond(rightMaster.getVelocity())
    );
  }

  public void resetOdemetry(Pose2d pose) {
    zeroSensors();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.curvatureDrive(xSpeed, zRotation, true);
    drive.feed();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    drive.feed();
  }

  public void driveMeters(double meters) {
    leftMaster.motionMagic(UnitConversion.distanceToNativeUnits(meters));
    rightMaster.motionMagic(UnitConversion.distanceToNativeUnits(meters));
  }

  public void driveRotations(double rotations) {
    leftMaster.motionMagic(rotationsToTicks(rotations));
    rightMaster.motionMagic(rotationsToTicks(rotations));
  }

  public void turn(double setAngle, double speed, double tolerance) {
    double angle = gyroPID.calculate(getGyroAngle(), setAngle);

		if (Math.abs(setAngle - getGyroAngle()) < tolerance) drive.tankDrive(0, 0);
		else drive.tankDrive(angle * speed, -angle * speed);
	}

  public int rotationsToTicks(double rotations) {
    return (int) rotations * 4096;
  }

  public double ticksToRotations(double ticks) {
    return ticks / 4096;
  }
  
  public boolean gyroPIDDone() {
    return gyroPID.atSetpoint();
  }

  public int getLeftVelocity() {
    return (int)leftMaster.getVelocity();
  }

  public int getRightVelocity() {
    return (int)rightMaster.getVelocity();
  }

  public int getLeftTicks() {
    return (int)leftMaster.getTicks();
  }

  public int getRightTicks() {
    return (int)rightMaster.getTicks();
  }

  public double getGyroAngle() {
    return RobotBase.isSimulation() ? gyro.getYaw() : -gyro.getYaw();
  }

  public void reset() {
    zeroSensors();
    resetGyro();
  }

  public void zeroSensors() {
    leftMaster.zeroSensor();
    rightMaster.zeroSensor();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public RamseteCommand createRamseteCommand(Trajectory path) {
    return new RamseteCommand(
      path, 
      this::getPose, 
      new RamseteController(), 
      new SimpleMotorFeedforward(
        DRIVETRAIN_ksVolts, 
        DRIVETRAIN_kvVoltSecondsPerMeter, 
        DRIVETRAIN_kaVoltSecondsSquaredPerMeter
      ),
      DRIVETRAIN_kKinematics, 
      this::getWheelSpeeds, 
      new PIDController(DRIVETRAIN_kPVelocity, 0, 0), 
      new PIDController(DRIVETRAIN_kPVelocity, 0, 0), 
      this::tankDriveVolts, 
      this
    );
  }

	public void changeGyroPID(double pGyro, double iGyro, double dGyro) {
    gyroPID.setPID(pGyro, iGyro, dGyro);
  }

  public void changeGyroPIDTolerance(double tolerance) {
    gyroPID.setTolerance(tolerance);
  }
  
  public static KitDrivetrain getInstance() {
    if (instance == null) {
      instance = new KitDrivetrain();
      instance.setDefaultCommand(new ArcadeDrive());
    }

		return instance;
  }
}
