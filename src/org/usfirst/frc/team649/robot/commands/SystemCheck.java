package org.usfirst.frc.team649.robot.commands;

import org.omg.stub.java.rmi._Remote_Stub;
import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class SystemCheck extends Command {

	int currentCommandState;
	boolean prevStateTrigger;
	
	boolean[] dT_results, shooter_results, pivot_results, intake_results;
	
	//
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		currentCommandState = 0;
		prevStateTrigger = false;
		
		
		while (! (Robot.oi.operatorJoystick.getRawButton(1) && !prevStateTrigger) ){ //wait for the click
			prevStateTrigger = Robot.oi.operatorJoystick.getRawButton(1);
		}
		
		dT_results = testDrivetrain();
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
//		switch(currentCommandState){
//			case 1:
//				
//				break;
//			case 2:
//				break;
//			case 3:
//				break;
//			case 4:
//				break;
//			case 5:
//				break;
//			default:
//				break;
//		}
		
	}

	//RESULTS: (0): success or failure, (1-infinity): subsystem dependent variables
	
	public boolean[] testDrivetrain(){
		boolean[] results = new boolean[3];
		Timer t = new Timer();
		
		double _gyro = Robot.drivetrain.gyro.getAngle(), _leftEnc = Robot.drivetrain.getDistanceDTLeft(), _rightEnc = Robot.drivetrain.getDistanceDTRight();
		
		t.reset();
		t.start();
		
		Robot.drivetrain.rawDrive(0.3, -0.3); //turn right
		while(t.get() < 0.4){
			
		}
		
		Robot.drivetrain.rawDrive(0, 0);
		
		double f_gyro = Robot.drivetrain.gyro.getAngle(), f_leftEnc = Robot.drivetrain.getDistanceDTLeft(), f_rightEnc = Robot.drivetrain.getDistanceDTRight();
		
		
		//gyro moved clockwise
		results[0] = Math.abs(f_gyro - _gyro) > 5.0;
					
		//left Enc moved forwards
		results[1] = f_leftEnc - _leftEnc > 10.0;
		
		//right Enc moved back
		results[2] = f_rightEnc - _rightEnc < -10.0;
		
		//log
		return results;
	}
	
	public boolean[] testShooters(){
		boolean[] results = new boolean[3];
		Timer t = new Timer();
		
		double _leftEinstein = Robot.shooter.getLeftFlywheelRPM(), _rightEinstein = Robot.shooter.getRightFlywheelRPM();
		
		t.reset();
		t.start();
		
		Robot.shooter.setLeftFlywheelPower(0.8);
		Robot.shooter.setRightFlywheelPower(0.8);
		
		while(t.get() < 1.0){
			//ramp up
		}
		
		Robot.drivetrain.rawDrive(0, 0);
		
		double f_leftEinstein = Robot.shooter.getLeftFlywheelRPM(), f_rightEinstein = Robot.shooter.getRightFlywheelRPM();
		
		
		//einstein spinning
		results[0] = Math.abs(f_leftEinstein - _leftEinstein) > 1000.0;
					
		//einstein spinning
		results[1] = Math.abs(f_rightEinstein - _rightEinstein) > 1000.0;
		
		Robot.shooter.setLeftFlywheelPower(0);
		Robot.shooter.setRightFlywheelPower(0);
		
		t.reset(); 
		t.start();
		
		while(t.get() < 0.6){
			//wait
		}
		
		//log
		return results;
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return currentCommandState == 8;
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
