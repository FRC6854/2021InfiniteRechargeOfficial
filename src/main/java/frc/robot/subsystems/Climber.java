package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.climber.DriveClimber;
import viking.controllers.ctre.VikingSPX;
import viking.controllers.ctre.VikingSRX;
import viking.controllers.rev.VikingMAX;

public class Climber extends SubsystemBase implements Constants, RobotMap {

	VikingMAX lift;
	VikingMAX massShifter;
	VikingSRX winchMaster;
	VikingSPX winchSlave;

	static Climber instance = null;

	public Climber() {
		lift = new VikingMAX(CAN_LIFT, true);
		massShifter = new VikingMAX(CAN_MASS_SHIFTER, false);

		winchMaster = new VikingSRX(CAN_WINCH_MASTER, false, false, FeedbackDevice.CTRE_MagEncoder_Relative, WINCH_kF, WINCH_kP, WINCH_kI, WINCH_kD, WINCH_MAX_VELOCITY, WINCH_ACCELERATION);
		winchSlave = new VikingSPX(CAN_WINCH_SLAVE, winchMaster, false);

		lift.setPIDF(LIFT_kP, LIFT_kI, LIFT_kD, LIFT_kF);
		lift.setSmartMotion(LIFT_MAX_VELOCITY, LIFT_ACCELERATION);

		// This sets the soft limits preventing the motor from driving to an unsafe position
		lift.enableSoftLimit(SoftLimitDirection.kForward, true);
		lift.setSoftLimit(SoftLimitDirection.kForward, LIFT_MAX_ROTATIONS);
		lift.enableSoftLimit(SoftLimitDirection.kReverse, true);
		lift.setSoftLimit(SoftLimitDirection.kReverse, 0.05f);
	}

	public void setLiftPosition(double ticks) {
		lift.positionControl(ticks);
	}

	public void setLiftOutput(double output) {
		lift.percentOutput(output);
	}

	public void setWinchOutput(double output) {
		if (output >= 0) {
			winchMaster.percentOutput(output);
		} else {
			winchMaster.percentOutput(0.0);
		}
	}

	public void setWinchPosition(double ticks) {
		winchMaster.positionControl(ticks);
	}

	public void setShifterOutput(double output) {
		massShifter.percentOutput(output);
	}

	public void fullStop() {
		lift.disable();
		massShifter.disable();
	}

	public double getLiftOutput() {
		return lift.getOutput();
	}

	public double getLiftTicks() {
		return lift.getPosition();
	}

	public double getWinchOutput() {
		return winchMaster.getMotorOutputPercent();
	}

	public int getWinchTicks() {
		return (int)winchMaster.getTicks();
	}

	public void zeroLift() {
		lift.zeroEncoder();
	}

	public void zeroWinch() {
		winchMaster.zeroSensor();
	}

	public double getShifterOutput() {
		return massShifter.getOutput();
	}

	public static Climber getInstance() {
		if(instance == null) {
			instance = new Climber();
			instance.setDefaultCommand(new DriveClimber());
		}
		return instance;
	}
}
