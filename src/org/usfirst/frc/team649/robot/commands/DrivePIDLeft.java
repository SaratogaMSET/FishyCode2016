// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class  DrivePIDLeft extends Command {
	double distance;
	double tolerance = 0.25;
	public PIDController drivePID;
	
    public DrivePIDLeft(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	drivePID = Robot.leftDT.getPIDController();
    	
    	this.distance = distance;
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	Robot.isPIDActiveLeft = true;
    	double setpoint = Robot.leftDT.getPosition() + distance;
    	drivePID.setSetpoint(setpoint);
    	SmartDashboard.putNumber("setpoint left", setpoint);
    	drivePID.enable();

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {

        return drivePID.onTarget();
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	drivePID.disable();
    	Robot.isPIDActiveLeft = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    }
}