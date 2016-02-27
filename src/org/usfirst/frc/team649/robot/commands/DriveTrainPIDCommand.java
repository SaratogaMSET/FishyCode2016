package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTrainPIDCommand extends Command {
	double PIDDistance;
	double tolerance = 0.25;
	public PIDController leftPID,rightPID;
	
    public DriveTrainPIDCommand(double distance,PIDController pidLeft,PIDController pidRight) {
    	leftPID = pidLeft;
    	rightPID = pidRight;
    	PIDDistance = distance;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	leftPID.enable();
    	rightPID.enable();
    	leftPID.setSetpoint(PIDDistance);
    	rightPID.setSetpoint(PIDDistance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return leftPID.onTarget() && rightPID.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	leftPID.disable();
    	rightPID.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
