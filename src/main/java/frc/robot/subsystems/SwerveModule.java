/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem implements RobotMap {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Motors
    private CANSparkMax drive;
    private TalonSRX turn;
    
    // Drive Encoder
    public CANEncoder driveEncoder;

    // Encoder Ticks
    private float driveEncoderTicks;
    private float turnEncoderTicks; 

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
    }

    /**
     * Sets the modules encoder tick configuration
     * 
     * @param driveEncoderTicks Number of ticks per wheel revolution
     * @param turnEncoderTicks Number of ticks per steering module revolution
     */
    public void configureModule(float driveEncoderTicks, float turnEncoderTicks) {
        this.driveEncoderTicks = driveEncoderTicks;
        this.turnEncoderTicks = turnEncoderTicks;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
