/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import frc.robot.DashboardSender;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem implements RobotMap, DashboardSender {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Motor declaration
    private SwerveModule fr;
    private SwerveModule fl;
    private SwerveModule bl;
    private SwerveModule br;

    // Gyro declaration
    public AHRS gyro;

    public Drivetrain() {
        // Motor instantiation
        fr = new SwerveModule(frontRightDrive, frontRightTurn);
        fl = new SwerveModule(frontLeftDrive, frontLeftTurn);
        bl = new SwerveModule(backLeftDrive, backLeftTurn);
        br = new SwerveModule(backRightDrive, backRightTurn);
        
        // Gyro instantiation
        gyro = new AHRS(SerialPort.Port.kMXP);
        gyro.zeroYaw();
    }

    public void zeroGyro() {
        gyro.zeroYaw();
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
