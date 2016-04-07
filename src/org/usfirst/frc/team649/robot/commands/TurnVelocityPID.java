package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TurnVelocityPID extends Command {
	
	static double l_velocity;
	static double r_velocity;
	static int coef;
	static double powerLeft, powerRight;
	static double powerToAddLeft, powerToAddRight;
	static double TURN_POWER_CAP = 0.8;
	static double leftErrorAccum;
	static double rightErrorAccum;
	
	static double leftPrevError;
	static double rightPrevError;
	
	public TurnVelocityPID(double velocity, boolean direction) {
		// TODO Auto-generated constructor stub
		coef = direction ? 1 : -1;
		l_velocity = coef * velocity;
		r_velocity = -coef * velocity;
	}

	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		powerLeft = 0;
		powerRight = 0;
		leftErrorAccum = 0;
		rightErrorAccum = 0;
		leftPrevError = l_velocity - Robot.drivetrain.leftEncoder.getRate();
		rightPrevError = r_velocity - Robot.drivetrain.rightEncoder.getRate();
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		System.out.println("Rate L: "  + Robot.drivetrain.leftEncoder.getRate() + ", Rate R: " + Robot.drivetrain.rightEncoder.getRate());
		double eLeft = l_velocity - Robot.drivetrain.leftEncoder.getRate(), eRight = r_velocity - Robot.drivetrain.rightEncoder.getRate();
		
		//LEFT SIDE
//		if (eLeft > 2.0){
//			powerToAddLeft = 0.01;
//		}
//		else if (eLeft > 1.0){
//			powerToAddLeft = 0.005;
//		}
//		else if (eLeft >= 0){
//			powerToAddLeft = 0;
//		}
//		else if (eLeft < 0){
//			powerToAddLeft = -0.005;
//		}
		
//		
//		//
//		if (eRight < -2.0){
//			powerToAddRight = 0.01;	
//		}
//		else if (eRight > 0.7){
//			powerToAddRight = 0.005;
//		}
//		else if (eRight >= 0){
//			powerToAddRight = 0;
//		}
//		else if (eRight > 0){
//			powerToAddRight = -0.005;
//		}
		
		leftErrorAccum += eLeft;
		rightErrorAccum += eRight;

		powerToAddLeft = 0.01 * eLeft; //+ 0.0 * leftErrorAccum + 0.0 * (eLeft - leftPrevError);
		powerToAddRight = 0.01 * eRight; //+ 0.0 * rightErrorAccum + 0.0 * (eRight - rightPrevError);
		
		powerLeft += coef * powerToAddLeft;
		powerRight += coef * powerToAddRight;
		
		powerLeft = Math.abs(powerLeft) > TURN_POWER_CAP ? 
				(powerLeft > 0 ? TURN_POWER_CAP : -TURN_POWER_CAP) : powerLeft;
		powerRight = Math.abs(powerRight) > TURN_POWER_CAP ? 
				(powerRight > 0 ? TURN_POWER_CAP : -TURN_POWER_CAP) : powerRight;
		
		Robot.drivetrain.motors[0].set(coef * -powerRight);
		Robot.drivetrain.motors[1].set(coef * -powerRight);
		Robot.drivetrain.motors[2].set(coef * powerLeft);
		Robot.drivetrain.motors[3].set(coef * powerLeft);
		
		
		System.out.println("Power to add: l > " + powerToAddLeft + ", r > " + powerToAddRight + " ... POWER: left > " 
				+ powerLeft + " right > " + powerRight);
		
		leftPrevError = eLeft;
		rightPrevError = eRight;
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}

}
