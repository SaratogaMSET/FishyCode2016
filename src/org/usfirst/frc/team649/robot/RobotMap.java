package org.usfirst.frc.team649.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static class Drivetrain {
		//FR,BR,BL,BR
		public static final int[] MOTOR_PORTS = {0,1,2,3};
		public static final int LEFT_ENCODER_FORWARD_CHANNEL = 0;
		public static final int LEFT_ENCODER_REVERSE_CHANNEL = 0;
		public static final int RIGHT_ENCODER_FORWARD_CHANNEL = 0;
		public static final int RIGHT_ENCODER_REVERSE_CHANNEL = 0;
		
		
	}
	
	public static class ShooterPivot {
		//adjust all with actual values
		public static final int[] MOTOR_PORTS = {4,5};
		public static final int[] ENCODER1 = {0,1};
		public static final int[] ENCODER2 = {2,3};
	}
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}
