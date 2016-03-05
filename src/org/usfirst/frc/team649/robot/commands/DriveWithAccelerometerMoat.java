package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithAccelerometerMoat extends Command {
	
	double accel1; //two accels required to calculate change
	double accel2;
	boolean finished;
	int i;

    public DriveWithAccelerometerMoat() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	i = 0;
    	accel1 = 0;
    	accel2 = 0;
    	finished = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(i % 2 == 0) {
    		accel1 = Robot.drivetrain.accel.getZ();
    	}
    	else if(i % 2 != 0){
    		accel2 = Robot.drivetrain.accel.getZ();
    	}
    	finished = Math.abs(accel2 - accel1) < DrivetrainSubsystem.AutoConstants.ACCEL_CHANGE_THRESHOLD;
    	i++;
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.rawDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
