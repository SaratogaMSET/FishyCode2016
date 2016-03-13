package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.LeftDTPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWithEncoders extends Command {

	public double setpoint;
	public Timer timer;
	Command left, right;
	public double deltaTranslationalDistance;
	
    public TurnWithEncoders(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	deltaTranslationalDistance = (angle/360.0) * (25.125 * Math.PI);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer = new Timer();
    	timer.reset();
    	timer.start();
    	left = new DrivePIDLeft(deltaTranslationalDistance);
    	
    	right = new DrivePIDRight(-deltaTranslationalDistance);
    	left.start();
    	right.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return left.isRunning()||right.isRunning();
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
