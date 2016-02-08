package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RetractSubsystem extends Subsystem {

	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	public static double STOP_SPEED = 0;
	Victor[] rollers;
	private static DoubleSolenoid leftSolenoid;
	private static DoubleSolenoid rightSolenoid;

	public RetractSubsystem() {
		rollers = new Victor[4];
		leftSolenoid = new DoubleSolenoid(RobotMap.Intake.FWD_LEFT_CHANNEL,
				RobotMap.Intake.BACK_LEFT_CHANNEL);
		rightSolenoid = new DoubleSolenoid(RobotMap.Intake.FWD_RIGHT_CHANNEL,
				RobotMap.Intake.BACK_RIGHT_CHANNEL);
		for (int i = 0; i < rollers.length; i++) {
			rollers[i] = new Victor(RobotMap.Intake.MOTOR_PORTS[i]);
		}
	}

	public void setBackRolSpd(double speed) {
		rollers[0].set(-speed);
		rollers[1].set(speed);
	}

	public void setCenteringModuleSpeed(double speed) {
		rollers[2].set(-speed);
		rollers[3].set(speed);
	}

	public void setSolenoids(DoubleSolenoid.Value set) {
		leftSolenoid.set(set);
		rightSolenoid.set(set);

	}

	protected void initDefaultCommand() {

	}
}