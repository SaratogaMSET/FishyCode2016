package org.usfirst.frc.team649.robot.commands;

import java.text.DecimalFormat;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.PIDConstants;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;
import org.usfirst.frc.team649.robot.util.Center;

import com.sun.javafx.css.CalculatedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnWithVision extends Command {

	double coef;
	int countTimesOffInARow;
	public static double power;
	double lDistance;
	double original_diff, diff;
	double prev_diff, prev2_diff, prev3_diff, prev4_diff, prev5_diff;
	
	boolean noTarget;
	boolean done;
	boolean findDirection;
	
	double initial_enc;
	boolean onTarget;
	
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
		power = TurnConstants.VISION_TURN_POWER;
	}
	
	public TurnWithVision(double turnPower) {
		// TODO Auto-generated constructor stub
		requires(Robot.drivetrain);
		findDirection = true;
		noTarget = false;
		done = false;
		lDistance = 0;
		df = new DecimalFormat("#.#");
		power = turnPower;

	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		onTarget = false;
    	Robot.autoAiming = true;
		countTimesOffInARow = 0;
		Robot.drivetrain.resetEncoders();
		initial_enc = Robot.drivetrain.getDistanceDTLeft();
			
		noTarget = Robot.currCenter.equals(new Center(-1,-1));
		Timer f = new Timer();
		f.reset();
		f.start();
		
		while (f.get() < 1.0){} //wait for the vision to settle just in case
		
		f.reset();
		f.start();
		
		if (noTarget){
			while (noTarget && f.get() < 1.5){
				noTarget = Robot.currCenter.equals(new Center(-1,-1));
			}
			noTarget = Robot.currCenter.equals(new Center(-1,-1)); //if it is still bad
		}
		
		original_diff = Robot.currCenter.x - Robot.GOOD_X; //positive means turn right
		SmartDashboard.putNumber("dff", original_diff);
		//compenstate for tilted target
		if(original_diff > 0) {
			System.out.println("-comp");
			original_diff -= 10;
		} else {
			System.out.println("comp");
			original_diff +=10;
		}
		
		diff = original_diff;
		prev_diff = original_diff;
		prev2_diff = original_diff;
		prev3_diff = original_diff;
		prev4_diff = original_diff;
		prev5_diff = original_diff;

		if (findDirection){ //set the coef here if the default constructor was called
			coef = diff > 0 ? 1 : -1;
		}
		
		double angle = calcTurnAngleFromVisionOffset(diff); //in dregrees
		lDistance = 40 * coef * Robot.drivetrain.getTranslationalDistanceForTurn(angle); // + (angle > 0 ? 3 : -3);
		System.out.println("Init turning vision=> original_diff: " + df.format(original_diff) 
							+"\n					angle: " + df.format(angle)
							+"\n					lDistance: " + df.format(lDistance));
		
		if (!noTarget){
			Robot.drivetrain.rawDrive(-coef * power, coef * power);
		}
	}

	@Override
	protected void execute() {
		diff = Robot.currCenter.x - Robot.GOOD_X;
		//p = 0.25 - 0.8/d <----------> ALT: try: 
		
		double C_VAL = 1;
//		if (Math.abs(diff - prev5_diff) > 15){ //too fast speed
//			C_VAL = 0.6;
//		} else if (Math.abs(diff - prev5_diff) < 2){ //too slow
//			C_VAL = 1.2;
//		}
//		
		power = C_VAL * ( TurnConstants.VISION_TURN_POWER - Math.abs(TurnConstants.VISION_KP/diff) ) ; //as diff gets smaller, slow down slightly
		power = power > 0 ? power : 0; 
		
		if (Math.abs(diff) <= Math.abs(original_diff)){ ///only keep turning if we are getting closer
			// TODO Auto-generated method stub
			countTimesOffInARow = 0; //reset if we find target
			Robot.drivetrain.rawDrive(-coef * power, coef * power);
			
		}
		else{
			countTimesOffInARow++;
		}
		
		//end when we are within tolerance in x direction
		done = Robot.isCenterWithinTolerance(true, false);
		
		System.out.println("DIFF in execute: " + diff + ", diff in diff: " + Math.abs(diff - prev5_diff) + ", C_VAL: " + C_VAL);
		
		onTarget = Math.abs((lDistance + initial_enc) - Robot.drivetrain.getDistanceDTLeft()) < 1.0;
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		boolean finished = noTarget || done || power == 0 || lostTarget()
				|| onTarget;
		prev5_diff = prev4_diff;
		prev4_diff = prev3_diff;
		prev3_diff =prev2_diff;
		prev2_diff = prev_diff;
		prev_diff = diff;
		return finished;
	}

	@Override
	protected void end() {
		diff = Robot.currCenter.x - Robot.GOOD_X;
		// TODO Auto-generated method stub
		Robot.drivetrain.rawDrive(0, 0);
		if (!noTarget && !lostTarget()){
			System.out.println("Done Vision Turning: endedBCVision = " + done + ", FINAL DIFF: " + df.format(diff)
								+ "\n  onTarget: " + onTarget + ", power == 0? " + (power == 0));
		}
		else{
			System.out.println("ERROR: TRIED turning vision, but FAILED"
					+ "\nWas there no target: " + noTarget + ", did we lose target? :" + lostTarget());
		}
		Robot.autoAiming = false;
	}

	@Override
	protected void interrupted() {
		System.out.println("turn interupted");
		end();
	}
	
	public boolean lostTarget(){
		return countTimesOffInARow > 250; //iterates 25 times (~ 0.5 s)
	}
	
	//given pix off from center, find angle in degrees to turn
		public double calcTurnAngleFromVisionOffset(double pix_off){
			double angle =  180.0/Math.PI * Math.atan(
					Math.abs(pix_off) / Robot.GOOD_X 
					* Math.tan(Robot.FIELD_OF_VIEW/2.0)
					);
			return pix_off > 0 ? angle : -angle;
		}
	
	

}
