package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    @Override
	protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	if(i % 2 == 0) {
    		accel1 = Robot.drivetrain.accel.getZ();
    	}
    	else if(i % 2 != 0){
    		accel2 = Robot.drivetrain.accel.getZ();
    	}
    	finished = Math.abs(accel2 - accel1) < DrivetrainSubsystem.AutoConstants.ACCEL_CHANGE_THRESHOLD;
    	i++;
    	

    	SmartDashboard.putString("DT Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return finished;
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
    }
}
