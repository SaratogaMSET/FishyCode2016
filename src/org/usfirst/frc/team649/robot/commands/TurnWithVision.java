package org.usfirst.frc.team649.robot.commands;

import java.text.DecimalFormat;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.PIDConstants;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;
import org.usfirst.frc.team649.robot.util.Center;

import com.sun.javafx.css.CalculatedValue;

import edu.wpi.first.wpilibj.command.Command;

public class TurnWithVision extends Command {

	double coef;
	int countTimesOffInARow;
	double power;
	double lDistance;
	double original_diff, diff;
	
	boolean noTarget;
	boolean done;
	boolean findDirection;
	
	DecimalFormat df;
	
	
	public TurnWithVision(boolean startTurnRight){
		requires(Robot.drivetrain);
		this.coef = startTurnRight ? 1 : -1;
		noTarget = false;
		done = false;
		lDistance = 0;
		df = new DecimalFormat("#.#");
		findDirection = false;
	}
	
	public TurnWithVision() {
		// TODO Auto-generated constructor stub
		requires(Robot.drivetrain);
		findDirection = true;
		noTarget = false;
		done = false;
		lDistance = 0;
		df = new DecimalFormat("#.#");
	}
	
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		countTimesOffInARow = 0;
			
			
		noTarget = Robot.currCenter.equals(new Center(-1,-1));
		
		original_diff = Robot.currCenter.x - Robot.GOOD_X; //positive means turn right
		diff = original_diff;

		if (findDirection){ //set the coef here if the default constructor was called
			coef = diff > 0 ? 1 : -1;
		}
		
		double angle = calcTurnAngleFromVisionOffset(diff); //in dregrees
		lDistance = coef * Robot.drivetrain.getTranslationalDistanceForTurn(angle);
		System.out.println("Init turning vision=> original_diff: " + df.format(original_diff) 
							+"\n					angle: " + df.format(angle)
							+"\n					lDistance: " + df.format(lDistance));
		power = TurnConstants.VISION_TURN_POWER;
		
		if (!noTarget){
			Robot.drivetrain.rawDrive(coef * power, -coef * power);
		}
	}

	@Override
	protected void execute() {
		diff = Robot.currCenter.x - Robot.GOOD_X;
		//p = 0.25 - 0.8/d <----------> ALT: try: 
		power = TurnConstants.VISION_TURN_POWER
				- Math.abs(TurnConstants.VISION_KP/diff); //as diff gets smaller, slow down slightly
		power = power > 0 ? power : 0; 
		
		if (Math.abs(diff) <= Math.abs(original_diff)){ ///only keep turning if we are getting closer
			// TODO Auto-generated method stub
			countTimesOffInARow = 0; //reset if we find target
			Robot.drivetrain.rawDrive(coef * power, -coef * power);
			
		}
		else{
			countTimesOffInARow++;
		}
		
		//end when we are within tolerance in x direction
		done = Robot.isCenterWithinTolerance(true, false);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return noTarget || done || power == 0 || lostTarget()
				|| Math.abs(lDistance - Robot.drivetrain.getDistanceDTLeft()) < PIDConstants.ABS_TOLERANCE;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.drivetrain.rawDrive(0, 0);
		if (!noTarget && !lostTarget()){
			System.out.println("Done Vision Turning: FINAL DIFF: " + df.format(diff));
		}
		else{
			System.out.println("ERROR: TRIED turning vision, but FAILED"
					+ "\nWas there no target: " + noTarget + ", did we lose target? :" + lostTarget());
		}
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}
	
	public boolean lostTarget(){
		return countTimesOffInARow > 25; //iterates 25 times (~ 0.5 s)
	}
	
	//given pix off from center, find angle in degrees to turn
		public double calcTurnAngleFromVisionOffset(double pix_off){
			double angle = 180.0/Math.PI * Math.atan(
					Math.abs(pix_off) / Robot.GOOD_X 
					* Math.tan(Robot.FIELD_OF_VIEW/2.0)
					);
			return pix_off > 0 ? angle : -angle;
		}
	
	

}
