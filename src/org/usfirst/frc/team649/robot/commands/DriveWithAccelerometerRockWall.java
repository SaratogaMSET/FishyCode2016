package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	

public class DriveWithAccelerometerRockWall extends Command {
	double lowestAccelValue;
	double encoderValue;
	boolean distanceIsEnough;
	boolean offRamp;	

    public DriveWithAccelerometerRockWall() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	lowestAccelValue = 1;
    	distanceIsEnough = false;
    	offRamp = false;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	Robot.drivetrain.resetEncoders();

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	Robot.drivetrain.rawDrive(-0.75, -0.75);
    	encoderValue = Robot.drivetrain.getDistanceDTBoth();
    	if(encoderValue > 57){
        	Robot.drivetrain.rawDrive(-0.85, -0.85);
    		
			if(Robot.drivetrain.accel.getZ() < lowestAccelValue){
	    			lowestAccelValue = Robot.drivetrain.accel.getZ();
				System.out.println("Lowest Value" + lowestAccelValue);
	   		}
			if(lowestAccelValue < -4.0){
				distanceIsEnough = true;
				System.out.println("DONE BC of Accel Values");
			}
			
			if(distanceIsEnough && Robot.drivetrain.accel.getZ() > 0.8 && Robot.drivetrain.accel.getZ() < 1.3){
				offRamp = true;
			}
	    	if (encoderValue > 300){
	    		offRamp = true;
	    		System.out.println("DONE BC of Encoder Overide");
	    	}
    	}

    	SmartDashboard.putString("DT Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return offRamp;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	Robot.drivetrain.rawDrive(0.0, 0.0);
    	Robot.comp.start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	end();
    }
}






















///**
// *
// */
//public class DriveWithAccelerometerRockWall extends Command {
//	double highestAccelValue;
//	double lowestAccelValue;
//	double encoderValue;
//	boolean distanceIsEnough;
//	double currentSpeed;
////	Timer time;
//	
//    public DriveWithAccelerometerRockWall() {
//        // Use requires() here to declare subsystem dependencies
//        // eg. requires(chassis);
//    	requires(Robot.drivetrain);
////    	time = new Timer();
//    	highestAccelValue = 1;
//    	lowestAccelValue = 1;
//    	distanceIsEnough = false;
//    }
//
//    // Called just before this Command runs the first time
//    @Override
//	protected void initialize() {
//    	Robot.drivetrain.resetEncoders();
////    	time.start();
//    }
//
//    // Called repeatedly when this Command is scheduled to run
//    @Override
//	protected void execute() {
//    	Robot.drivetrain.rawDrive(-0.75, -0.75);
//    	encoderValue = Robot.drivetrain.getDistanceDTBoth();
//    	if(encoderValue > 20){
//    		Robot.comp.stop();
////    		if(Robot.drivetrain.accel.getZ() > highestAccelValue){
////    			highestAccelValue = Robot.drivetrain.accel.getZ();
////    		}else if(Robot.drivetrain.accel.getZ() < lowestAccelValue){
////    			lowestAccelValue = Robot.drivetrain.accel.getZ();
////    		}
////    		if(highestAccelValue-lowestAccelValue > 11.0){
////    			distanceIsEnough = true;
////    		}
//    		if (encoderValue > 260){
//    			distanceIsEnough = true;
//    			System.out.println("DONE");
//    		}
//    	}
//
//    	SmartDashboard.putString("DT Current Command", this.getName());
////    	System.out.println("Accel: " + (highestAccelValue - lowestAccelValue) + ", Ending soon?" + distanceIsEnough);
//    }
//
//    // Make this return true when this Command no longer needs to run execute()
//    @Override
//	protected boolean isFinished() {
//        return distanceIsEnough;
//    }
//
//    // Called once after isFinished returns true
//    @Override
//	protected void end() {
//    	Robot.drivetrain.rawDrive(0.0, 0.0);
//    	Robot.comp.start();
//    }
//
//    // Called when another command which requires one or more of the same
//    // subsystems is scheduled to run
//    @Override
//	protected void interrupted() {
//    	end();
//    }
//}
