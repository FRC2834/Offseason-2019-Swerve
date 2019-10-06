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
    public static int frontRightDrive = 5;
    public static int frontRightTurn = 1;

    public static int frontLeftDrive = 2;
    public static int frontLeftTurn = 3;

    public static int backLeftDrive = 4;
    public static int backLeftTurn = 5;

    public static int backRightDrive = 6;
    public static int backRightTurn = 7;

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

    static final double nominalTurnOutputPercent = 0.75;
    static final int encoderTickAtNominal = 803;
    
    static final double fGain = (nominalTurnOutputPercent * 1023) / encoderTickAtNominal;

    static final double pGain = 0.2;
    static final double error = 90;
    static final double pTerm = (pGain * 1023) / error;

    static final double dGain = 10*pTerm;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    static final Gains kGains = new Gains(pTerm, 0.001, dGain, fGain, 100, 0.0);
}
