/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;
import jaci.pathfinder.modifiers.SwerveModifier.Mode;
import frc.robot.DashboardSender;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem implements RobotMap, DashboardSender {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Motor declaration
    public SwerveModule fr;
    public SwerveModule fl;
    public SwerveModule bl;
    public SwerveModule br;

    // Gyro declaration
    public AHRS gyro;

    // Trajectory config
    Trajectory.Config config;

    public Drivetrain() {
        // Motor instantiation
        fr = new SwerveModule(frontRightDrive, frontRightTurn, -2132+2048);
        fl = new SwerveModule(frontLeftDrive, frontLeftTurn, -2270+2048);
        bl = new SwerveModule(backLeftDrive, backLeftTurn, -3284+2048);
        br = new SwerveModule(backRightDrive, backRightTurn, -166-2048);

        fr.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        fl.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        br.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        bl.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        
        // Gyro instantiation
        gyro = new AHRS(SerialPort.Port.kMXP);
        gyro.zeroYaw();

        // Trajectory config instantation
        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, max_velocity, max_acceleration, max_jerk);
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public double[][] getMotorInputs() {
        return null;
    }

    public void controlModule(SwerveModule module, double speed, double angle) {
        module.move(speed, angle);
    }

    public SwerveModifier generateSwerveTrajectory(Waypoint[] waypoints) {
        // Generate trajectory
        Trajectory trajectory = Pathfinder.generate(waypoints, config);
        // Modify for swerve
        SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;
        SwerveModifier modifier = new SwerveModifier(trajectory);
        modifier.modify(WHEEL_BASE_METERS, TRACK_WIDTH_METERS, mode);

        return modifier;
    }

    public EncoderFollower[] getFollowers(SwerveModifier modifier) {
        // Get followers
        EncoderFollower frFollower = new EncoderFollower(modifier.getFrontRightTrajectory());
        frFollower.configureEncoder(fr.getDriveEncoderPosition(), 188.4, wheel_diameter);
        frFollower.configurePIDVA(kp, ki, kd, kv, ka);

        EncoderFollower flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory());
        flFollower.configureEncoder(fl.getDriveEncoderPosition(), 188.4, wheel_diameter);
        flFollower.configurePIDVA(kp, ki, kd, kv, ka);

        EncoderFollower blFollower = new EncoderFollower(modifier.getBackLeftTrajectory());
        blFollower.configureEncoder(bl.getDriveEncoderPosition(), 188.4, wheel_diameter);
        blFollower.configurePIDVA(kp, ki, kd, kv, ka);

        EncoderFollower brFollower = new EncoderFollower(modifier.getBackRightTrajectory());
        brFollower.configureEncoder(br.getDriveEncoderPosition(), 188.4, wheel_diameter);
        brFollower.configurePIDVA(kp, ki, kd, kv, ka);

        EncoderFollower[] followers = new EncoderFollower[] {
            frFollower, flFollower, blFollower, brFollower
        };

        return followers;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new Drive());
    }

	@Override
	public void dashboardInit() {
        SmartDashboard.putNumber("gyro yaw", gyro.getYaw());

        SmartDashboard.putNumber("fr encoder", fr.getEncoderPosition());
        SmartDashboard.putNumber("fl encoder", fl.getEncoderPosition());
        SmartDashboard.putNumber("bl encoder", bl.getEncoderPosition());
        SmartDashboard.putNumber("br encoder", br.getEncoderPosition());

        SmartDashboard.putNumber("fr offset", fr.moduleOffset);
        SmartDashboard.putNumber("fl offset", fl.moduleOffset);
        SmartDashboard.putNumber("bl offset", bl.moduleOffset);
        SmartDashboard.putNumber("br offset", br.moduleOffset);
	}

	@Override
	public void dashboardPeriodic() {
        SmartDashboard.putNumber("gyro yaw", gyro.getYaw());
        SmartDashboard.putNumber("fr encoder", fr.getEncoderPosition());
        SmartDashboard.putNumber("fl encoder", fl.getEncoderPosition());
        SmartDashboard.putNumber("bl encoder", bl.getEncoderPosition());
        SmartDashboard.putNumber("br encoder", br.getEncoderPosition());

        SmartDashboard.putNumber("fr offset", fr.moduleOffset);
        SmartDashboard.putNumber("fl offset", fl.moduleOffset);
        SmartDashboard.putNumber("bl offset", bl.moduleOffset);
        SmartDashboard.putNumber("br offset", br.moduleOffset);
	}
}
