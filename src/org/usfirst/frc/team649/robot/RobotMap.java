package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int OPERATOR_JOYSTICK = 0;
	public static final int DRIVE_LEFT_JOYSTICK = 1;
	public static final int DRIVE_RIGHT_JOYSTICK = 2;
	
	public static class Drivetrain {
		// FR,BR,BL,BR
		public static final int[] MOTOR_PORTS = { 0, 1, 2, 3 };
		public static final int[] DRIVE_SOLENOID_PORTS = {0,3,1,3};
		public static final int LEFT_ENCODER_FORWARD_CHANNEL = 0;
		public static final int LEFT_ENCODER_REVERSE_CHANNEL = 1;
		public static final int RIGHT_ENCODER_FORWARD_CHANNEL = 2;
		public static final int RIGHT_ENCODER_REVERSE_CHANNEL = 3;
	}

	public static class Intake {
		public static final int[] MOTOR_PORTS = { 17, 18, 19};
		public static final int[] LEFT_SOLENOID_PORTS = {0,6,1,6};
		public static final int[] RIGHT_SOLENOID_PORTS = {0,7,1,7};
	}

	public static class ShooterPivot {
		// adjust all with actual values
		public static final int[] MOTOR_PORTS = { 4, 5 };
		public static final int[] ENCODER1 = { 0, 1 };
		public static final int[] ENCODER2 = { 2, 3 };
		public static final int HALL_EFFECT_SENSOR = 0;
		public static final int [] LEFT_SOLENOID_PORTS = {0,3,1,3};
		public static final int [] RIGHT_SOLENOID_PORTS = {0,4,1,4};
	}


}
