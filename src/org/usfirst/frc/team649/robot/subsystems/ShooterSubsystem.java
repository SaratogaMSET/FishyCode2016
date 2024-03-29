package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {
	
	public Victor leftMotor, rightMotor;
	public DoubleSolenoid loader;
	public Counter leftPhotoelectric, rightPhotoelectric;
	public DigitalInput infraredSensor;
	
	public static int IN = 0;
	public static int OUT = 1;
	public static int OFF = 2;
	
	public static boolean UNTIL_IR = true;
	
	/*****CONSTANTS FOR FLYWHEEL****/
	public static final double FARSHOT_FLYWHEEL_TARGET_RPM = 3700;
	public static final double FARSHOT_FLYWHEEL_MAX_SHOOT_POWER = 0.85;
	public static final double FARSHOT_FLYWHEEL_MIN_SHOOT_POWER = 0.72;
	public static final double FARSHOT_FLYWHEEL_TOLERANCE = 100;
	
	public static final double BATTER_FLYWHEEL_TARGET_RPM = 2050;
	public static final double BATTER_FLYWHEEL_MAX_SHOOT_POWER = 0.5;
	public static final double BATTER_FLYWHEEL_MIN_SHOOT_POWER = 0.3;
	public static final double BATTER_FLYWHEEL_TOLERANCE = 60;

	public static final double DEFENSE_FLYWHEEL_TARGET_RPM = 2515;
	public static final double DEFENSE_FLYWHEEL_MAX_SHOOT_POWER = 0.54;
	public static final double DEFENSE_FLYWHEEL_MIN_SHOOT_POWER = 0.48;
	public static final double DEFENSE_FLYWHEEL_TOLERANCE = 40;
	
	/*****EDITABLE FLYWHEEL RPM THAT IS UPDATED IN TELEOP*****/
	public static double flywheelTargetRPM = 3500;
	public static double flywheelMaxShootPower = .8;
	public static double flywheelMinShootPower = 0.6;
	public static double flywheelTolerance = 90;
	
	//doubles as purge speed when pivot is down (just inverted of course)
	public static final double FLYWHEEL_INTAKE_POWER = -0.6;
	
	//intaking bang bang
	public static final double INTAKE_TARGET_RPM = 3000;
	public static final double INTAKE_MAX_SHOOT_POWER = -0.8;
	public static final double INTAKE_MIN_SHOOT_POWER = -0.5;
	
	public ShooterSubsystem() {
		super("shooter subsystem");
		leftMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[0]);
		rightMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[1]);
	
		loader = new DoubleSolenoid(RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[0],
				RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[1], RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[2]);
		
		leftPhotoelectric = new Counter(RobotMap.ShooterSubsystem.LEFT_EINSTEIN);
		leftPhotoelectric.setReverseDirection(true);;
		
		rightPhotoelectric = new Counter(RobotMap.ShooterSubsystem.RIGHT_EINSTEIN);
		rightPhotoelectric.setReverseDirection(false);
		

		infraredSensor = new DigitalInput(RobotMap.ShooterSubsystem.IR_GATE_PORT);
	}

	public double getLeftFlywheelRPM() {
		return 60/leftPhotoelectric.getPeriod();
	}

	public double getRightFlywheelRPM() {
		return 60/rightPhotoelectric.getPeriod();
	}
	
	public void setLeftFlywheelPower(double pwr) {
		leftMotor.set(pwr);
		SmartDashboard.putNumber("Left Flywheel power", pwr);
	}
	
	public void setRightFlywheelPower(double pwr) {
		rightMotor.set(pwr);
		SmartDashboard.putNumber("Right Flywheel power", pwr);
	}
	
	//bang bang
	public void setFlywheelPowerBangBang(double max_pwr, double min_pwr, double target_rpm) {
//		System.out.println("TARGET RPM: " + target_rpm);
		if(getLeftFlywheelRPM() <= target_rpm){
			setLeftFlywheelPower(max_pwr);
		}	
		else{
			setLeftFlywheelPower(min_pwr);
		}
		
		if(getRightFlywheelRPM() <= target_rpm){
			setRightFlywheelPower(-max_pwr);
		}
		else{
			setRightFlywheelPower(-min_pwr);
		}
	}
	
	
	public void loadBall(Value punchPower) {
		loader.set(punchPower);
	}

	public boolean isIRTripped(){
		return !infraredSensor.get();
	}
	
	public void resetCounter() {
		rightPhotoelectric.reset();
		leftPhotoelectric.reset();
	}
	
	public void updateShooterConstants(){
		//if close-ish to batter shot
		double TOLERANCE = 5;
		if (Robot.shooterPivot.getPivotAngle() > PivotPID.CLOSE_SHOOT_POSITION - TOLERANCE 
				&& Robot.shooterPivot.getPivotAngle() < PivotPID.CLOSE_SHOOT_POSITION + TOLERANCE){
			flywheelTargetRPM = BATTER_FLYWHEEL_TARGET_RPM;
			flywheelMinShootPower = BATTER_FLYWHEEL_MIN_SHOOT_POWER;
			flywheelMaxShootPower = BATTER_FLYWHEEL_MAX_SHOOT_POWER;
			flywheelTolerance =  BATTER_FLYWHEEL_TOLERANCE;
		}
		else if (Robot.shooterPivot.getPivotAngle() > PivotPID.ANTI_DEFENSE_POSITION - TOLERANCE 
				&& Robot.shooterPivot.getPivotAngle() < PivotPID.ANTI_DEFENSE_POSITION + TOLERANCE){
			flywheelTargetRPM = DEFENSE_FLYWHEEL_TARGET_RPM;
			flywheelMinShootPower = DEFENSE_FLYWHEEL_MIN_SHOOT_POWER;
			flywheelMaxShootPower = DEFENSE_FLYWHEEL_MAX_SHOOT_POWER;
			flywheelTolerance =  DEFENSE_FLYWHEEL_TOLERANCE;
		}
		//else assume it is far shot
		else{
			flywheelTargetRPM = FARSHOT_FLYWHEEL_TARGET_RPM;
			flywheelMinShootPower = FARSHOT_FLYWHEEL_MIN_SHOOT_POWER;
			flywheelMaxShootPower = FARSHOT_FLYWHEEL_MAX_SHOOT_POWER;
			flywheelTolerance =  FARSHOT_FLYWHEEL_TOLERANCE;
		}
	}
	
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
