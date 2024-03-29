package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class  DrivetrainPIDCommand extends Command {
	double distance;
	double tolerance = 1.0;
	public PIDController drivePID;
	
    public DrivetrainPIDCommand(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	
    	drivePID = Robot.drivetrain.getPIDController();
    	//drivePIDRight = Robot.drivetrain.getPIDController();
    	this.distance = distance;
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	drivePID.enable();
    	//drivePIDRight.enable();
    	Robot.isPIDActive = true;
    	double setpoint = Robot.drivetrain.getPosition() + distance;
    	drivePID.setSetpoint(setpoint);
    	//drivePIDRight.setSetpoint(setpoint);
    	System.out.println("DT PID: setpoint = " + setpoint);
    	
    	Robot.logMessage("DT driving with PID, initial Enc: " + Robot.drivetrain.getPosition() + ", moving to: " + setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	SmartDashboard.putString("DT Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {


        return drivePID.onTarget()
        		|| Robot.oi.driver.isManualOverride() || Robot.drivetrain.isOnTarget(distance);
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	drivePID.disable();
    	Robot.drivetrain.rawDrive(0, 0);
    	//drivePIDRight.disable();
    	Robot.isPIDActive = false;

    	Robot.logMessage("DT ended at: " + Robot.drivetrain.getDistanceDTBoth());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}