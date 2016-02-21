package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Counter;
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

	public static final double FLYWHEEL_TARGET_RPM = 1900;
	public static final double FLYWHEEL_MAX_POWER = 0.4;
	public static final double FLYWHEEL_MIN_POWER = 0.32;
	public static final double FLYWHEEL_TOLERANCE = 90;
	
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
	
	public void loadBall(Value punchPower) {
		loader.set(punchPower);
	}

	public void resetCounter() {
		rightPhotoelectric.reset();
		leftPhotoelectric.reset();
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
