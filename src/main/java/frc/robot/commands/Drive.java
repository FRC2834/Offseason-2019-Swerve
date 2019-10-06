/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveModule;

public class Drive extends Command {
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double FWD = Robot.m_oi.driver.getRawAxis(1);
    if(Math.abs(FWD) < 0.1) {
      FWD = 0;
    }
    double STR = Robot.m_oi.driver.getRawAxis(0);
    if(Math.abs(STR) < 0.1) {
      STR = 0;
    }
    double RCW = Robot.m_oi.driver.getRawAxis(4);
    if(Math.abs(RCW) < 0.1) {
      RCW = 0;
    }

    double[][] vectors = SwerveModule.calculate(-FWD, 
    STR, 
    RCW, 
    0.0, 
    Robot.drivetrain.fr.getBaseLength(), 
    Robot.drivetrain.fr.getBaseWidth());

    Robot.drivetrain.controlModule(Robot.drivetrain.fr, vectors[0][0], vectors[0][1]);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
