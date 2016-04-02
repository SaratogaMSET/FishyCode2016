package org.usfirst.frc.team649.robot.runnables;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class SystemCheckThread implements Runnable {

	int currentCommandState;
	boolean prevStateTrigger;
	
	Map<String,Boolean> dT_results, shooter_results, pivot_results, intake_results;
	
	@Override
	public void run() {
		
		// TODO Auto-generated method stub
		currentCommandState = 0;
		System.out.println("STARTED SYSTEM CHECK");
		
		waitForButton();
		currentCommandState++;
		System.out.println("button 1");
		
		if (Robot.robotEnabled) testDrivetrain();
		
		printOutResults(dT_results);
		
		waitForButton();
		currentCommandState++;
		System.out.println("button 2");
		
		//if (Robot.robotEnabled) testShooters();
		
		waitForButton();
		currentCommandState++;
		System.out.println("button 3");
		
		//if (Robot.robotEnabled) testIntakes();
		
		waitForButton();
		currentCommandState++;
		System.out.println("button 4");
		
		//if (Robot.robotEnabled) testPivot();
		
		currentCommandState = 100;
	}
	

	public void waitForButton(){
		prevStateTrigger = false;
		while (Robot.oi.operatorJoystick.getRawButton(1) && Robot.robotEnabled){
			//WAITING in case button was originally held down and not released in time
//			System.out.println("WAITING FOR UNPRESS");
		}
		while (!Robot.oi.operatorJoystick.getRawButton(1) && Robot.robotEnabled){ //wait for the click
			//System.out.println("prev: " + prevStateTrigger + ", current: " + Robot.oi.operatorJoystick.getRawButton(1));
			
			
			prevStateTrigger = Robot.oi.operatorJoystick.getRawButton(1);
		}
		System.out.println("DONE WAITING FOR BUTTON");
	}
	
	public void printOutResults(Map<String, Boolean> results){
		Iterator it = results.entrySet().iterator();
		while (it.hasNext()){
			Map.Entry pair = (Map.Entry)it.next();
			System.out.println((String)pair.getKey() + ", " + ((Boolean)pair.getValue()).booleanValue());
		}
	}
	
	////**********************************TEST SUBSYSTEMS**********************************////
	
	//RESULTS: (0): success or failure, (1-infinity): subsystem dependent variables
	
	
	/// <<<<<<<<<<<<<<DRIVETRAIN>>>>>>>>>>>>>
	
	public void testDrivetrain(){
		dT_results = new HashMap<>();
		Timer t = new Timer();
		
		double _gyro = Robot.drivetrain.gyro.getAngle(), _leftEnc = Robot.drivetrain.getDistanceDTLeft(), _rightEnc = Robot.drivetrain.getDistanceDTRight();
		
		t.reset();
		t.start();
		
		Robot.drivetrain.rawDrive(0.3, -0.3); //turn right
		while(t.get() < 0.8){
			
		}
		
		Robot.drivetrain.rawDrive(0, 0);
		
		double f_gyro = Robot.drivetrain.gyro.getAngle(), f_leftEnc = Robot.drivetrain.getDistanceDTLeft(), f_rightEnc = Robot.drivetrain.getDistanceDTRight();
		
		
		//gyro moved clockwise
		dT_results.put("DT GYRO WORKING?", Math.abs(f_gyro - _gyro) > 5.0);
					
		//left Enc moved forwards
		dT_results.put("DT LEFT ENCODER WORKING?", f_leftEnc - _leftEnc > 3.0);
		
		//right Enc moved back
		dT_results.put("DT RIGHT ENCODER WORKING?", f_rightEnc - _rightEnc < -3.0);
		
	}
	
	/// <<<<<<<<<<<<<<SHOOTERS>>>>>>>>>>>>>
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
			//wait for slowing down
		}
		
		//log
		return results;
	}
	
	/// <<<<<<<<<<<<<<INTAKES>>>>>>>>>>>>>
	public boolean[] testIntakes(){
		boolean[] results = new boolean[3];
		Timer t = new Timer();
		t.reset(); 
		t.start(); 
		boolean _intakeDeployed = Robot.intake.isIntakeDeployed();
		
		if (_intakeDeployed){
//			Robot.intake.setSolenoids(IntakeSubsystem.UP);
//			while (t.get() < 2.0){
//				//wait
//			}
			results[0] = true; //because we know intakes work if they started true
		}
		else{
			Robot.intake.setSolenoids(!IntakeSubsystem.UP);
			while (t.get() < 2.3){
				//wait
			}
			results[0] = Robot.intake.isIntakeDeployed();
		}
		
		t.reset();
		t.start();
		
		Robot.intake.setCenteringModuleSpeed(IntakeSubsystem.CENTERING_MODULE_INTAKE_SPEED);
		Robot.intake.setFwdRolSpd(IntakeSubsystem.FORWARD_ROLLER_INTAKE_SPEED);
		//turn right
		while(!Robot.shooter.isIRTripped() || t.get() > 12){
			 
		}
		
		Robot.intake.setCenteringModuleSpeed(0);
		Robot.intake.setFwdRolSpd(0);
		
		results[1] = t.get() <= 12 && Robot.shooter.isIRTripped(); 
		
		
		//log
		return results;
	}
	
	// <<<<<<<<<<<<<<<<<<<<<<<<PIVOT>>>>>>>>>>>>>>>>>>>>>>>>
	public boolean[] testPivot() {
		boolean[] results = new boolean[3];
		Timer t = new Timer();
		t.reset(); 
		t.start();
		
		Robot.shooterPivot.setPower(ShooterPivotSubsystem.PivotPID.MAX_MOTOR_DOWN_POWER);
		while (!Robot.shooterPivot.isResetHalTripped() && t.get() < 5.0){
			System.out.println("waiting for pivot to trip");
		}
		Robot.shooterPivot.setPower(0);
		
		results[0] = t.get() <= 5.0 && Robot.shooterPivot.isResetHalTripped();
		
		t.reset();
		t.start();

		double _encLeft = Robot.shooterPivot.encoderLeft.getDistance(), _encRight = Robot.shooterPivot.encoderRight.getDistance();
		
		Robot.shooterPivot.setPower(ShooterPivotSubsystem.PivotPID.REGULAR_MAX_UP_POWER);
		if (t.get() > 1.0){
			//going up
		}
		Robot.shooterPivot.setPower(0);
		
		double f_encLeft = Robot.shooterPivot.encoderLeft.getDistance(), f_encRight = Robot.shooterPivot.encoderRight.getDistance();
		
		results[1] = f_encLeft - _encLeft > 3;
		results[2] = f_encRight - _encRight > 3;
		
		//log
		return results;
	}


}
