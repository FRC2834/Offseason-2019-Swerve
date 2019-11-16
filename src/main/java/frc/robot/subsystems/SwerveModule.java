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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DashboardSender;
import frc.robot.RobotMap;

import java.lang.Math;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem implements RobotMap, DashboardSender {
    // Motors
    public CANSparkMax drive;
    public TalonSRX turn;
    
    // Drive encoder
    public CANEncoder driveEncoder;

    // Encoder ticks for each motor
    private double driveEncoderTicks;
    private double turnEncoderTicks;

    // Steering motor zero in encoder ticks
    private double moduleZero;

    // Module offset from steering motor zero position
    public double moduleOffset;

    // Module zeroed?
    public boolean zeroed;

    // Wheelbase dimensions
    private double baseLength;

    // Trackwidth dimensions
    private double baseWidth;

    /**
     * Constructor for a swerve module
     * 
     * @param driveID The ID for the drive motor
     * @param turnID The ID for the steering motor
     * @param moduleZero Value of analog encoder at steering module forward position (May need to flip the sign based on the sensor phase)
     */
    public SwerveModule(int driveID, int turnID, int moduleZero) {
        // Instantiate motors with CAN ID
        drive = new CANSparkMax(driveID, MotorType.kBrushless);
        turn = new TalonSRX(turnID);

        // Instantiate module zero
        this.moduleZero = moduleZero;
        zeroed = false;

        // Instantiate drive encoder
        driveEncoder = new CANEncoder(drive);

        // Configure drive motor
        drive.setOpenLoopRampRate(0.1);
        drive.setSmartCurrentLimit(50);

        // Reset the encoder settings
        turn.configFactoryDefault();
        
        // Select absolute encoder to use
        turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);

        // Set sensor and motor direction
        turn.setSensorPhase(true);
        turn.setInverted(false);

        // Set relevant frame periods to be at least as fast as periodic rate
		turn.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		turn.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		// Set the peak and nominal outputs
		turn.configNominalOutputForward(0, kTimeoutMs);
		turn.configNominalOutputReverse(0, kTimeoutMs);
		turn.configPeakOutputForward(1, kTimeoutMs);
		turn.configPeakOutputReverse(-1, kTimeoutMs);

		// Set Motion Magic gains in slot0
		turn.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		turn.config_kF(kSlotIdx, kGains.kF, kTimeoutMs);
		turn.config_kP(kSlotIdx, kGains.kP, kTimeoutMs);
		turn.config_kI(kSlotIdx, kGains.kI, kTimeoutMs);
        turn.config_kD(kSlotIdx, kGains.kD, kTimeoutMs);
        turn.config_IntegralZone(kSlotIdx, kGains.kIzone, kTimeoutMs);

		// Set acceleration and vcruise velocity
		turn.configMotionCruiseVelocity(MODULE_CRUISE_VELOCITY, kTimeoutMs);
        turn.configMotionAcceleration(MODULE_ACCELERATION, kTimeoutMs);  
    }

    /**
     * Configure module parameters
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
     * Gest the distance the module is from the zero in ticks
     */
    public void calculateOffset() {
        moduleOffset = turn.getSelectedSensorPosition() - moduleZero;
    }

    /**
     * Returns the current position of the encoder in ticks
     * 
     * @return The position of the encoder in ticks
     */
    public int getEncoderPosition() {
        return turn.getSelectedSensorPosition();
    }

    /**
     * Calculates module angles and speed
     * 
     * @param FWD Joystick y input
     * @param STR Joystick x input
     * @param RCW Joystick 2 x input
     * @param gyroAngle Gyro angle used for field-centric swerve drive
     * @param baseLength Length of wheelbase
     * @param baseWidth Width of wheelbase
     * @return Returns a 2d array containing the speed and angle in degrees for each module
     */
    public static double[][] calculate(double FWD, double STR, double RCW, double gyroAngle, double baseLength, double baseWidth) {
        // Maks the command field-centric
        double temp = FWD * Math.cos(Math.toRadians(gyroAngle)) + STR * Math.sin(Math.toRadians(gyroAngle));
        STR = -FWD * Math.sin(Math.toRadians(gyroAngle)) + STR * Math.cos(Math.toRadians(gyroAngle));
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

    /**
     * Returns the nearest coterminal zero in ticks
     * 
     * @param currentPosition The current position of the steering module in ticks
     * @param moduleZero The modules zero
     * @return The nearest coterminal zero in ticks
     */
    public double getNearestCoterminalZero(int currentPosition, double moduleZero) {
        double coterminalZero = moduleZero;
        if(currentPosition > 0) {
            while(coterminalZero < currentPosition) {
                coterminalZero += turnEncoderTicks;
            }
        } else if(currentPosition < 0) {
            while(coterminalZero > currentPosition) {
                coterminalZero -= turnEncoderTicks;
            }
        }
        return coterminalZero;
    }

    /**
     * Moves the motor to the nearest coterminal zero
     * Call before running the robot
     */
    public void zeroModule() {
        zeroed = false;
        do {
            turn.set(ControlMode.MotionMagic, getNearestCoterminalZero(turn.getSelectedSensorPosition(), moduleZero));
        } while(turn.getClosedLoopError(kPIDLoopIdx) > 5);
        zeroed = true;
    }

    /**
     * Moves the steering module to a target
     * 
     * @param speed The speed at which to move the drive motor
     * @param targetAngle The target angle of the steering module in degrees
     */
    public void move(double speed, double targetAngle) {

        // Derive the alternate target angle
        double targetAngle2 = targetAngle + 180;

        // Make the angle input from the -180 - 180 range to the 0 - 360 range
        if(targetAngle < 0) {
            targetAngle += 360;
        }

        // Get current angle and normalize it to a 0 - 360 range
        double currentAngle = (turn.getSelectedSensorPosition() - moduleZero) * (360 / turnEncoderTicks);
        while(currentAngle > 360) {
            currentAngle -= 360;
        }
        while(currentAngle < 0) {
            currentAngle += 360;
        }
        
        // Calculate shortest angle to target
        double delta1 = 0;
        double delta2 = 0;
        if(currentAngle > targetAngle) {
            // cw delta
            delta1 = 360 - currentAngle + targetAngle;
            // ccw delta
            delta2 = -(currentAngle - targetAngle);  
        } else if(targetAngle > currentAngle) {
            // cw delta
            delta1 = targetAngle - currentAngle;
            // ccw delta
            delta2 = -(360 - targetAngle + currentAngle);
        }

        double delta = 0;
        if(Math.abs(delta1) < Math.abs(delta2)) {
            delta = delta1;
        } else {
            delta = delta2;
        }

        double delta3 = 0;
        double delta4 = 0;
        if(currentAngle > targetAngle2) {
            // cw delta
            delta3 = 360 - currentAngle + targetAngle2;
            // ccw delta
            delta4 = -(currentAngle - targetAngle2); 
        } else if(targetAngle2 > currentAngle) {
            // cw delta
            delta3 = targetAngle2 - currentAngle;
            // ccw delta
            delta4 = -(360 - targetAngle2 + currentAngle);
        }

        double altDelta = 0;
        if(Math.abs(delta3) < Math.abs(delta4)) {
            altDelta = delta3;
        } else {
            altDelta = delta4;
        }

        // Moves the drive motor and steering module if the module has been zeroed
        if(true) {
            if(Math.abs(delta) < Math.abs(altDelta)) {
                double ticksDelta = delta * (turnEncoderTicks / 360);
                turn.set(ControlMode.MotionMagic, turn.getSelectedSensorPosition() + ticksDelta);
                //turn.set(ControlMode.MotionMagic, moduleZero);
                SmartDashboard.putNumber("delta", delta);
                if(turn.getClosedLoopError(kPIDLoopIdx) < 50) {
                    drive.set(speed);
                }
            } else if(Math.abs(altDelta) < Math.abs(delta)) {
                double ticksDelta = altDelta * (turnEncoderTicks / 360);
                turn.set(ControlMode.MotionMagic, turn.getSelectedSensorPosition() + ticksDelta);
                //turn.set(ControlMode.MotionMagic, moduleZero);
                SmartDashboard.putNumber("delta", altDelta);
                if(turn.getClosedLoopError(kPIDLoopIdx) < 50) {
                    drive.set(-speed);
                }
            } else {
                turn.set(ControlMode.MotionMagic, turn.getSelectedSensorPosition());
                //turn.set(ControlMode.MotionMagic, moduleZero);
            }
        }
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
