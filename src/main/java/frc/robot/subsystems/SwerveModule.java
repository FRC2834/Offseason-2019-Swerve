/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.DashboardSender;
import frc.robot.RobotMap;

import java.lang.Math;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem implements RobotMap, DashboardSender {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Motors
    private CANSparkMax drive;
    private TalonSRX turn;
    
    // Drive Encoder
    public CANEncoder driveEncoder;

    // Encoder Ticks
    private double driveEncoderTicks;
    private double turnEncoderTicks;

    // Wheelbase dimensions
    private double baseLength;
    private double baseWidth;

    /**
     * Constructor for a swerve module
     * 
     * @param driveID The ID for the drive motor
     * @param turnID The ID for the steering motor
     */
    public SwerveModule(int driveID, int turnID) {
        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new TalonSRX(turnID);

        driveEncoder = new CANEncoder(drive);

        // Reset the encoder settings
        turn.configFactoryDefault();
        // Select encoder to use
        turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

        // Set sensor and motor direction
        turn.setSensorPhase(true);
        turn.setInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
		turn.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		turn.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		turn.configNominalOutputForward(0, kTimeoutMs);
		turn.configNominalOutputReverse(0, kTimeoutMs);
		turn.configPeakOutputForward(1, kTimeoutMs);
		turn.configPeakOutputReverse(-1, kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		turn.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		turn.config_kF(kSlotIdx, kGains.kF, kTimeoutMs);
		turn.config_kP(kSlotIdx, kGains.kP, kTimeoutMs);
		turn.config_kI(kSlotIdx, kGains.kI, kTimeoutMs);
		turn.config_kD(kSlotIdx, kGains.kD, kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		turn.configMotionCruiseVelocity(15000, kTimeoutMs);
		turn.configMotionAcceleration(6000, kTimeoutMs);

		/* Zero the sensor */
        turn.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }

    /**
     * Sets the modules encoder tick configuration
     * 
     * @param driveEncoderTicks Number of ticks per wheel revolution
     * @param turnEncoderTicks Number of ticks per steering module revolution
     * @param baseLength Length of wheelbase
     * @param baseWidth Width of wheelbase
     */
    public void configureModule(double driveEncoderTicks, double turnEncoderTicks, double baseLength, double baseWidth) {
        this.driveEncoderTicks = driveEncoderTicks;
        this.turnEncoderTicks = turnEncoderTicks;
        this.baseLength = baseLength;
        this.baseWidth = baseWidth;
    }

    public double getBaseLength() { return baseLength; }
    public double getBaseWidth() { return baseWidth; }

    /**
     * Calculates module angles and speed
     * 
     * @param FWD Joystick y input
     * @param STR Joystick x input
     * @param RCW Joystick 2 x input
     * @param gyroAngle Gyro angle used for field-centric swerve drive
     * @param baseLength Length of wheelbase
     * @param baseWidth Width of wheelbase
     * @return Returns a 2d array containing the speed and angle for each module
     */
    public static double[][] calculate(double FWD, double STR, double RCW, double gyroAngle, double baseLength, double baseWidth) {
        // Makes the command field-centric
        double temp = FWD * Math.cos(gyroAngle) + STR * Math.sin(gyroAngle);
        STR = -FWD * Math.sin(gyroAngle) + STR * Math.cos(gyroAngle);
        FWD = temp;

        double R = Math.sqrt(Math.pow(baseLength, 2) + Math.pow(baseWidth, 2));

        double A = STR - RCW * (baseLength / R);
        double B = STR + RCW * (baseLength / R);
        double C = FWD - RCW * (baseWidth / R);
        double D = FWD + RCW * (baseWidth / R);

        // Wheel Speeds
        double fr_ws = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double fl_ws = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double bl_ws = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double br_ws = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        // Wheel Angles measured clockwise, with zero being straight, from -180 to +180 degrees
        double fr_wa = Math.atan2(B, C) * 180 / Math.PI;
        double fl_wa = Math.atan2(B, D) * 180 / Math.PI;
        double bl_wa = Math.atan2(A, D) * 180 / Math.PI;
        double br_wa = Math.atan2(A, C) * 180 / Math.PI;

        // Normalize wheel speeds
        double max = fr_ws;

        if(fl_ws > max) {
            max = fl_ws;
        } else if(bl_ws > max) {
            max = bl_ws;
        } else if(br_ws > max) {
            max = br_ws;
        }

        if(max > 1) {
            fr_ws /= max;
            fl_ws /= max;
            bl_ws /= max;
            br_ws /= max;
        }

        double[][] output = new double[][] {
            {fr_ws, fr_wa}, 
            {fl_ws, fl_wa},
            {bl_ws, bl_wa}, 
            {br_ws, br_wa}
        };

        return output;
    }

    public void move(double speed, double angle) {
        drive.set(speed);
        turn.set(ControlMode.MotionMagic, angle);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void dashboardInit() {

    }

    @Override
    public void dashboardPeriodic() {

    }
}
