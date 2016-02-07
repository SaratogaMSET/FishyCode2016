package org.usfirst.frc.team649.robot.subsystems.intake;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeSubsystem extends Subsystem {

	public static double INTAKE_SPEED = 1.0;
	public static double PURGE_SPEED = -1.0;
	public static double STOP_SPEED = 0;
	Victor[] rollers;
	private static DoubleSolenoid leftSolenoids;
	private static DoubleSolenoid rightSolenoids;

	public IntakeSubsystem() {
		rollers = new Victor[4];
		leftSolenoids = new DoubleSolenoid(RobotMap.Intake.FWD_LEFT_CHANNEL,
				RobotMap.Intake.BACK_LEFT_CHANNEL);
		rightSolenoids = new DoubleSolenoid(RobotMap.Intake.FWD_RIGHT_CHANNEL,
				RobotMap.Intake.BACK_RIGHT_CHANNEL);
		for (int i = 0; i < rollers.length; i++) {
			rollers[i] = new Victor(RobotMap.Intake.MOTOR_PORTS[i]);
		}
	}

	public void setFwdRolSpd(double speed) {
		rollers[0].set(speed);
		rollers[1].set(-speed);
	}

	public void setBackRolSpd(double speed) {
		rollers[2].set(-speed);
		rollers[3].set(speed);
	}

	public static boolean setSolenoids(boolean b) {
		if (!b) {
			ReverseSolenoids();
		}
		if (b) {
			ForwardSolenoids();
		}
		return b;

	}

	public static void ReverseSolenoids() {
		leftSolenoids.set(DoubleSolenoid.Value.kReverse);
		rightSolenoids.set(DoubleSolenoid.Value.kReverse);
	}

	public static void ForwardSolenoids() {
		leftSolenoids.set(DoubleSolenoid.Value.kForward);
		rightSolenoids.set(DoubleSolenoid.Value.kForward);
	}

	protected void initDefaultCommand() {

	}
}