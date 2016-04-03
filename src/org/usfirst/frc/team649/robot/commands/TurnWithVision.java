package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.command.Command;

public class TurnWithVision extends Command {

	boolean startLeft;
	double distance;
	double diff;
	
	boolean noTarget;
	boolean done;
	
	
	public TurnWithVision(boolean startTurnLeft, double distance){
		this.startLeft = startTurnLeft;
		noTarget = false;
		done = false;
		distance = this.distance;
	}
	
	public double calcTurnAngleFromVisionOffset(double pix_off, double distance){
		return 0;
	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		noTarget = Robot.currCenter.equals(new Center(-1,-1));
		double initial_x = Robot.currCenter.x;
		diff = initial_x - Robot.GOOD_X; //positive means turn right
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		if (startLeft){ //turn left
			Robot.drivetrain.rawDrive(-TurnConstants.VISION_TURN_POWER, TurnConstants.VISION_TURN_POWER);
		}
		else{
			Robot.drivetrain.rawDrive(TurnConstants.VISION_TURN_POWER, -TurnConstants.VISION_TURN_POWER);
		}
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return noTarget;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.drivetrain.rawDrive(0, 0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}
	
	

}
