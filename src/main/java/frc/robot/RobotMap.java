package frc.robot;

public interface RobotMap {
    /**
     * ROBOT MAP
     * 
     * Constants for physical conections on robot
     * 
     * FORMAT: (data type can be changed as needed)
     * public static final int CONECTOR_LOCATION/SUBSYSTEM = NUMBER
     * 
     * to reference data from RobotMap include
     * 
     * "implements RobotMap" on the class declaration ex. 
     * public class CoolRobot implements RobotMap
     * 
     * DO NOT HARDCODE NUMBERS IN CODE. ALWAYS UPDATE & REFERENCE TO THIS FILE OR CONSTANTS. 
     * MAGIC NUMBERS SUCK!!!!
     * 
     */

    /**
     * ROBOT MAP
     * 
     *   Shooter
     *    BACK
     * ||||||||||
     * ||||||||||
     * ||||||||||
     * ||||||||||
     * |||    |||
     *    FRONT
     *  Conveyor
     */
    
     /**
     * Controllers: Number in drivestation NOTE: Position in ds list can be locked
     * inside drivestation Make sure this is done before each match and check before
     * each match that the order is correct.
     */
    public static final int CONTROLLER_DRIVER = 0;
    public static final int CONTROLLER_OPERATOR = 1;
    
    /**
     * Motor Controllers: Specify the connection type (CAN, PWM) and
     * location/subsystem in the constant
     */

    // DRIVETRAIN
    public static final int CAN_LEFT_FRONT = 20;
    public static final int CAN_RIGHT_FRONT = 13;
    public static final int CAN_LEFT_BACK = 12;
    public static final int CAN_RIGHT_BACK = 10;

    // SHOOTER
    public static final int CAN_TOP_SHOOTER = 5;
    public static final int CAN_BOTTOM_SHOOTER = 2;

    // CONVEYORS
    public static final int CAN_INTAKE_CONVEYOR = 1;
    public static final int CAN_UPPER_CONVEYOR = 4;

    // CLIMBER
    public static final int CAN_WINCH_MASTER = 2;
    public static final int CAN_WINCH_SLAVE = 1;
    public static final int CAN_LIFT = 7;
    public static final int CAN_MASS_SHIFTER = 8;

}