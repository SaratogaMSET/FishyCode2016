package org.usfirst.frc.team649.robot.subsystems;

import java.rmi.server.RMIClassLoader;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {
	
	public Victor leftMotor, rightMotor;
	public DoubleSolenoid loader;
	public Counter leftPhotoeletric, rightPhotoeletric;

	
	public ShooterSubsystem() {
		super("shooter subsystem");
		leftMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[0]);
		rightMotor = new Victor(RobotMap.ShooterSubsystem.MOTOR_PORTS[1]);
	
		loader = new DoubleSolenoid(RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[0],
				RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[1], RobotMap.ShooterSubsystem.PUNCH_SOLENOID_PORTS[2]);
		
		leftPhotoeletric = new Counter(RobotMap.ShooterSubsystem.LEFT_EINSTEIN);
		leftPhotoeletric.setReverseDirection(true);;
		
		rightPhotoeletric = new Counter(RobotMap.ShooterSubsystem.RIGHT_EINSTEIN);
		rightPhotoeletric.setReverseDirection(false);
	}

	public double getLeftFlywheelRPM() {
		return 60/leftPhotoeletric.getPeriod();
	}

	public double getRightFlywheelRPM() {
		return 60/rightPhotoeletric.getPeriod();
	}
	
	public void setLeftFlywheelPower(double pwr) {
		leftMotor.set(pwr);
	}
	
	public void setRightFlywheelPower(double pwr) {
		rightMotor.set(pwr);
	}
	
	public void loadBall(Value punchPower) {
		loader.set(punchPower);
	}

	public void resetCounter() {
		rightPhotoeletric.reset();
		leftPhotoeletric.reset();
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
