package org.usfirst.frc.team649.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int OPERATOR_JOYSTICK = 0;
	
	public static class Drivetrain {
		// FR,BR,BL,BR
		public static final int[] MOTOR_PORTS = { 0, 1, 2, 3 };
		public static final int LEFT_ENCODER_FORWARD_CHANNEL = 0;
		public static final int LEFT_ENCODER_REVERSE_CHANNEL = 0;
		public static final int RIGHT_ENCODER_FORWARD_CHANNEL = 0;
		public static final int RIGHT_ENCODER_REVERSE_CHANNEL = 0;
	}

	public static class Intake {
		public static final int[] MOTOR_PORTS = { 4, 5, 6, 7 };
		public static final int FWD_LEFT_CHANNEL = 0;
		public static final int FWD_RIGHT_CHANNEL = 0;
		public static final int BACK_LEFT_CHANNEL = 0;
		public static final int BACK_RIGHT_CHANNEL = 0;
		public static final int SOLENOID_FORWARD_CHANNEL = 0;
		public static final int SOLENOID_REVERSE_CHANNEL = 0;
	}

	public static class ShooterPivot {
		// adjust all with actual values
		public static final int[] MOTOR_PORTS = { 4, 5 };
		public static final int[] ENCODER1 = { 0, 1 };
		public static final int[] ENCODER2 = { 2, 3 };
		public static final int HALL_EFFECT_SENSOR = 0;
	}


}
