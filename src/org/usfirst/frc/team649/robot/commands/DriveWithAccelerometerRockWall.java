package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	
/**
 *
 */
public class DriveWithAccelerometerRockWall extends Command {
	double highestAccelValue;
	double lowestAccelValue;
	double encoderValue;
	boolean distanceIsEnough;
	double currentSpeed;
	Timer time;
	
    public DriveWithAccelerometerRockWall() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	time = new Timer();
    	highestAccelValue = 1;
    	lowestAccelValue = 1;
    	distanceIsEnough = false;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	Robot.drivetrain.rawDrive(.75, .75);
    	time.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	encoderValue = Robot.drivetrain.getDistanceDTBoth();
    	if(time.get()>0.75 && encoderValue > 60){
    		if(Robot.drivetrain.accel.getZ() > highestAccelValue){
    			highestAccelValue = Robot.drivetrain.accel.getZ();
    		}else if(Robot.drivetrain.accel.getZ() < lowestAccelValue){
    			lowestAccelValue = Robot.drivetrain.accel.getZ();
    		}
    		if(highestAccelValue-lowestAccelValue > 0.4){
    			distanceIsEnough = true;
    		}
    	}

    	SmartDashboard.putString("DT Current Command", this.getName());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return distanceIsEnough;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.drivetrain.rawDrive(0.0, 0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    }
}
