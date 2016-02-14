package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

public class IntakeSubsystem extends Subsystem {

	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	public static double STOP_SPEED = 0;
	Victor[] rollers;
	private DoubleSolenoid649 leftSolenoid;
	private DoubleSolenoid649 rightSolenoid;

	public IntakeSubsystem() {
		rollers = new Victor[4];
		leftSolenoid = new DoubleSolenoid649(RobotMap.Intake.LEFT_SOLENOID_PORTS[0],
				RobotMap.Intake.LEFT_SOLENOID_PORTS[1],RobotMap.Intake.LEFT_SOLENOID_PORTS[2],
				RobotMap.Intake.LEFT_SOLENOID_PORTS[3]);
		rightSolenoid = new DoubleSolenoid649(RobotMap.Intake.RIGHT_SOLENOID_PORTS[0],
				RobotMap.Intake.RIGHT_SOLENOID_PORTS[1],RobotMap.Intake.RIGHT_SOLENOID_PORTS[2],
				RobotMap.Intake.RIGHT_SOLENOID_PORTS[3]);
		for (int i = 0; i < rollers.length; i++) {
			rollers[i] = new Victor(RobotMap.Intake.MOTOR_PORTS[i]);
		}
	}

	public void setFwdRolSpd(double speed) {
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}

	public void setCenteringModuleSpeed(double speed) {
		rollers[2].set(-speed);
		rollers[3].set(speed);
	}

	public void setSolenoids(boolean set) {
		leftSolenoid.set(set);
		rightSolenoid.set(set);

	}

	protected void initDefaultCommand() {

	}
}