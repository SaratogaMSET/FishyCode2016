package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

public class IntakeSubsystem extends Subsystem {

	public static final double FORWARD_ROLLER_INTAKE_SPEED = 0.35;
	public static final double CENTERING_MODULE_INTAKE_SPEED = 1.0;
	public static final double FORWARD_ROLLER_PURGE_SPEED = -1.0;
	public static final double CENTERING_MODULE_PURGE_SPEED = -1.0;
	public static final double INTAKE_OFF_SPEED = 0.0;
	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	public static double STOP_SPEED = 0;
	
	public static double MIN_STALL_TIME = 1.0;
	public static double CURRENT_THRESHOLD = 15;
	
	Victor[] rollers;
	public DoubleSolenoid intakeSolenoid;
	//public DoubleSolenoid rightSolenoid;

	public IntakeSubsystem() {
		rollers = new Victor[3];
		intakeSolenoid = new DoubleSolenoid(RobotMap.Intake.LEFT_SOLENOID_PORTS[0],
				RobotMap.Intake.LEFT_SOLENOID_PORTS[1],RobotMap.Intake.LEFT_SOLENOID_PORTS[2]);
//		rightSolenoid = new DoubleSolenoid(RobotMap.Intake.RIGHT_SOLENOID_PORTS[0],
//				RobotMap.Intake.RIGHT_SOLENOID_PORTS[1],RobotMap.Intake.RIGHT_SOLENOID_PORTS[2]);
		for (int i = 0; i < rollers.length; i++) {
			rollers[i] = new Victor(RobotMap.Intake.MOTOR_PORTS[i]);
		}
	}

	public void setFwdRolSpd(double speed) {
		rollers[2].set(speed);
	}

	public void setCenteringModuleSpeed(double speed) {
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}
	//true = up, false = down
	public void setSolenoids(boolean set) {
		if (set){
			intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			//rightSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		else{
			intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
			//rightSolenoid.set(DoubleSolenoid.Value.kReverse);
		}

	}
	
	public boolean getSolenoids() {
		if(intakeSolenoid.get() == DoubleSolenoid.Value.kForward){ //|| rightSolenoid.get() == DoubleSolenoid.Value.kForward) {
			return true; 
		}
		return false;
	}

	public boolean isIntakeDeployed() {
		//TODO, update when sol's for intake are actually plugged in
		return getSolenoids();
	}
	
	protected void initDefaultCommand() {

	}
}