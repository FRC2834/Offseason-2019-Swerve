/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;

    // Motor IDs
    public static final int frontRightDrive = 1;
    public static final int frontRightTurn = 5;

    public static final int frontLeftDrive = 2;
    public static final int frontLeftTurn = 6;

    public static final int backLeftDrive = 3;
    public static final int backLeftTurn = 7;

    public static final int backRightDrive = 4;
    public static final int backRightTurn = 8;

    // Motor Encoder Ticks
    public static final double DRIVE_ENCODER_TICKS = 40.0;
    public static final double TURN_ENCODER_TICKS = 4096.0;

    // Chassis Dimensions
    public static final double TRACK_WIDTH = 22.625;
    public static final double WHEEL_BASE = 22.625;

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

    // static final double nominalTurnOutputPercent = 0.75;
    // static final int encoderTickAtNominal = 803;

    static final double nominalTurnOutputPercent = 0.95;
    static final int encoderTickAtNominal = 1245;
    
    static final double fGain = (nominalTurnOutputPercent * 1023) / encoderTickAtNominal;

    static final double pGain = 0.23;
    static final double error = 193;
    static final double pTerm = (pGain * 1023) / error;

    static final double dGain = 10*pTerm;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(pTerm, 0.005, dGain, fGain, 120, 0.0);

    // Steering module cruise velocity and acceleration
    public static final int MODULE_CRUISE_VELOCITY = 1245;
    public static final int MODULE_ACCELERATION = 3735;
}
