/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

public class FollowPath extends Command {

  // Encoder followers
  EncoderFollower[] followers;
  EncoderFollower frFollower;
  EncoderFollower flFollower;
  EncoderFollower blFollower;
  EncoderFollower brFollower;

  double frTarget;
  double flTarget;
  double blTarget;
  double brTarget;

  double frHeading;
  double flHeading;
  double blHeading;
  double brHeading;

  public FollowPath(EncoderFollower[] followers) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    this.followers = followers;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    frFollower = followers[0];
    flFollower = followers[1];
    blFollower = followers[2];
    brFollower = followers[3];
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    frTarget = frFollower.calculate((int) Robot.drivetrain.fr.getDriveEncoderPosition());
    frHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(frFollower.getHeading()));

    Robot.drivetrain.controlModule(Robot.drivetrain.fr, frTarget, frHeading);
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
