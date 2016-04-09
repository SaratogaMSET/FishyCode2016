package org.usfirst.frc.team649.robot.commands;

import java.text.DecimalFormat;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnWithVision extends Command {

	double coef;
	int countTimesOffInARow;
	static double l_velocity;
	static double r_velocity;
	static double abs_vel;
	static double powerLeft, powerRight;
	static double powerToAddLeft, powerToAddRight;
	static double TURN_POWER_CAP = 0.8;
	double lDistance;
	double original_diff, diff;
	
	boolean noTarget;
	boolean centerOnTarget;
	boolean findDirection;
	boolean waitInBeginning;
	
	double initial_enc;
	boolean dtEncOnTarget;
	
	DecimalFormat df;
	
	
//	public TurnWithVision(boolean startTurnRight){
////		requires(Robot.drivetrain);
//		this.coef = startTurnRight ? 1 : -1;
//		noTarget = false;
//		done = false;
//		lDistance = 0;
//		df = new DecimalFormat("#.#");
//		findDirection = false;
//		
//		powerLeft = 0;
//		powerRight = 0;
//	}
//	
	public TurnWithVision() {
		// TODO Auto-generated constructor stub
//		requires(Robot.drivetrain);
		findDirection = true;
		noTarget = false;
		centerOnTarget = false;
		waitInBeginning = true;
		lDistance = 0;
		df = new DecimalFormat("#.#");
		
		powerLeft = 0;
		powerRight = 0;
		
		abs_vel = 8;
	}
	
	public TurnWithVision(double vel, boolean waitStart) {
		// TODO Auto-generated constructor stub
//		requires(Robot.drivetrain);
		findDirection = true;
		noTarget = false;
		centerOnTarget = false;
		waitInBeginning = waitStart;
		lDistance = 0;
		df = new DecimalFormat("#.#");
		
		powerLeft = 0;
		powerRight = 0;
		
		abs_vel = vel;

	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		dtEncOnTarget = false;
		countTimesOffInARow = 0;
		Robot.drivetrain.resetEncoders();
		initial_enc = Robot.drivetrain.getDistanceDTLeft();
			
		noTarget = Robot.currCenter.equals(new Center(-1,-1));
		Timer f = new Timer();
		f.reset();
		f.start();
		
		if (waitInBeginning){
			while (f.get() < 1.0){} //wait for the vision to settle just in case
		}
		
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
			original_diff -= 1;
		} else {
			System.out.println("comp");
			original_diff += 3;
		}
//		
		diff = original_diff;

		if (findDirection){ //set the coef here if the default constructor was called
			coef = diff > 0 ? 1 : -1;
			l_velocity = coef * abs_vel;
			r_velocity = -coef * abs_vel;
		}
		
		double angle = calcTurnAngleFromVisionOffset(diff); //in dregrees
		lDistance = 40 * coef * Robot.drivetrain.getTranslationalDistanceForTurn(angle); // + (angle > 0 ? 3 : -3);
		System.out.println("Init turning vision=> original_diff: " + df.format(original_diff) 
							+"\n					angle: " + df.format(angle)
							+"\n					lDistance: " + df.format(lDistance));
		
//		if (!noTarget){
//			Robot.drivetrain.rawDrive(-coef * power, coef * power);
//			
//		}
		Robot.logMessage("START TurnWithVision, originalDiff: " + df.format(original_diff) + ", calculated angle: " + df.format(angle) + ", noTarget: " + noTarget);
	}

	@Override
	protected void execute() {
		diff = Robot.currCenter.x - Robot.GOOD_X;
		//p = 0.25 - 0.8/d <----------> ALT: try: 
//		
//		power = C_VAL * ( TurnConstants.VISION_TURN_POWER - Math.abs(TurnConstants.VISION_KP/diff) ) ; //as diff gets smaller, slow down slightly
//		power = power > 0 ? power : 0; 
//		double d = 0;
		if (Math.abs(diff) < 30){
			if (original_diff > 40){
				l_velocity *= 0.6; //decrease speed by 30 % if
				r_velocity *= 0.6;
			}
		}
		
//		if (Math.abs(diff) <= Math.abs(original_diff)){ ///only keep turning if we are getting closer
			double eLeft = l_velocity - Robot.drivetrain.leftEncoder.getRate(), eRight = r_velocity - Robot.drivetrain.rightEncoder.getRate();
			
			
			// TODO Auto-generated method stub
//			Robot.drivetrain.rawDrive(-coef * power, coef * power);
			
			powerToAddLeft = 0.01 * eLeft; //+ 0.0 * leftErrorAccum + 0.0 * (eLeft - leftPrevError);
			powerToAddRight = 0.01 * eRight; //+ 0.0 * rightErrorAccum + 0.0 * (eRight - rightPrevError);
			
			powerLeft += coef * powerToAddLeft; //+ coef * d;
			powerRight += coef * powerToAddRight; //+ coef * d;
			
//			powerLeft = powerLeft > 0 ? powerLeft: powerLeft;
//			powerRight = powerRight > 0 ? powerRight: powerRight;
			
			//capping the speed
			powerLeft = Math.abs(powerLeft) > TURN_POWER_CAP ? 
					(powerLeft > 0 ? TURN_POWER_CAP : -TURN_POWER_CAP) : powerLeft;
			powerRight = Math.abs(powerRight) > TURN_POWER_CAP ? 
					(powerRight > 0 ? TURN_POWER_CAP : -TURN_POWER_CAP) : powerRight;
			
			//setting the speeds
			Robot.drivetrain.motors[0].set(coef * -powerRight);
			Robot.drivetrain.motors[1].set(coef * -powerRight);
			Robot.drivetrain.motors[2].set(coef * powerLeft);
			Robot.drivetrain.motors[3].set(coef * powerLeft);
			
			System.out.println("Power to add: l > " + powerToAddLeft + ", r > " + powerToAddRight + " ... POWER: left > " 
					+ powerLeft + " right > " + powerRight);
			
//		}
		
		if (Robot.currCenter.x == -1){
			countTimesOffInARow++;
		}
		else{
			countTimesOffInARow = 0;
		}
		
		//end when we are within tolerance in x direction
		centerOnTarget = Robot.isCenterWithinTolerance(true, false);
		
		System.out.println("DIFF in execute: " + diff +", d: " + (0.01 * diff - 0.3));
		
		dtEncOnTarget = Math.abs((lDistance + initial_enc) - Robot.drivetrain.getDistanceDTLeft()) < 1.0;
		
		Robot.logMessage("EXECUTE TurnWithVision, diff: " + df.format(diff) + ", centerOnTarget " + centerOnTarget + ", countTimesOff: " + countTimesOffInARow + ", Power add: l > " + powerToAddLeft + ", r > " + powerToAddRight + " ... POWER: left > " + powerLeft + " right > " + powerRight);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		boolean finished = noTarget || centerOnTarget || lostTarget()
				|| dtEncOnTarget || Robot.oi.driver.isManualOverride();
		return finished;
	}

	@Override
	protected void end() {
		diff = Robot.currCenter.x - Robot.GOOD_X;
		// TODO Auto-generated method stub
		Robot.drivetrain.rawDrive(0, 0);
		if (!noTarget && !lostTarget()){
			System.out.println("Done Vision Turning: endedBCVision = " + centerOnTarget + ", FINAL DIFF: " + df.format(diff)
								+ "\n  onTarget: " + dtEncOnTarget);
			Robot.logMessage("ENDED TurnWithVision: endedBCVision = " + centerOnTarget + ", FINAL DIFF: " + df.format(diff) + "\n  onTarget: " + dtEncOnTarget);
		}
		else{
			System.out.println("ERROR: TRIED turning vision, but FAILED"
					+ "\nWas there no target: " + noTarget + ", did we lose target? :" + lostTarget());
			Robot.logMessage("ERROR: TRIED TURNING VISION BUT FAILED, TurnWithVision -> Was there no target: " + noTarget + ", did we lose target :" + lostTarget());
		}
//		Robot.autoAiming = false;
	}

	@Override
	protected void interrupted() {
		System.out.println("turn interupted");
		Robot.logMessage("INTERRUPTED TurnWithVision");
		end();
	}
	
	public boolean lostTarget(){
		return countTimesOffInARow > 250;
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
