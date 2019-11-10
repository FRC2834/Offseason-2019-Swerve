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

    public Drivetrain() {
        // Motor instantiation
        fr = new SwerveModule(frontRightDrive, frontRightTurn, -101);
        fl = new SwerveModule(frontLeftDrive, frontLeftTurn, -302);
        bl = new SwerveModule(backLeftDrive, backLeftTurn, -3210);
        br = new SwerveModule(backRightDrive, backRightTurn, -2248);

        fr.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        fl.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        br.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        bl.configureModule(DRIVE_ENCODER_TICKS, TURN_ENCODER_TICKS, WHEEL_BASE, TRACK_WIDTH);
        
        // Gyro instantiation
        gyro = new AHRS(SerialPort.Port.kMXP);
        gyro.zeroYaw();
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
