package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnWithEncoders extends Command {

	public double setpoint;
	public Timer timer;
	DrivePIDLeft left;
	DrivePIDRight right;
	public double deltaTranslationalDistance;
	public boolean prevStateLeftPID, prevStateRightPID;
	
    public TurnWithEncoders(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	deltaTranslationalDistance = Robot.drivetrain.getTranslationalDistanceForTurn(angle);
    }

    // Called just before this Command runs the first time
    @Override
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
    @Override
	protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        boolean done = !left.isRunning() && !right.isRunning();
        SmartDashboard.putBoolean("Done?", done);
        prevStateLeftPID = Robot.isPIDActiveLeft;
        prevStateRightPID = Robot.isPIDActiveRight;
        return done;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.drivetrain.rawDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}
