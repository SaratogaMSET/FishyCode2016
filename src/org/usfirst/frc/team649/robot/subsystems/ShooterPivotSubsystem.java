package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class ShooterPivotSubsystem extends PIDSubsystem {
	
	public Victor motorLeft;
	public Victor motorRight;
	public Encoder encoderLeft;
	public Encoder encoderRight;
	public PIDController pidLeft, pidRight;
	public Counter counter; //unsure about validity of counter/hall effect
	public DoubleSolenoid649 leftSol;
	public DoubleSolenoid649 rightSol;
	
	public static class PivotPID {
		
		private static final double ENCODER_DISTANCE_PER_PULSE = 0; //change of course
		public static final double SHOOTER_P = 0.15;
		public static final double SHOOTER_I = 0.15;
		public static final double SHOOTER_D = 0.15;
		public static final double ABS_TOLERANCE = 0.05;
		public static final double MAX_MOTOR_POWER = 0.5;
		public static final double MIN_MOTOR_POWER = 0.15;
		public static double SHOOTER_POWER = 0.1; //idk
		public static final int PICKUP_STATE = 0;
		public static final int STORING_STATE = 1;
		public static final int RELEASE_STATE = 2;
		public static final double PIVOT_PICKUP = 2.02;//arbitrary value
		public static final double PIVOT_STORING = 3.05;//arbitrary value
		public static final double PIVOT_RELEASE = 1.8;//arbitrary value
		
		
		
	}


    public ShooterPivotSubsystem() {

    	super("shooter pivot", PivotPID.SHOOTER_P, PivotPID.SHOOTER_I, PivotPID.SHOOTER_D);
    	motorLeft = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[0]);
    	motorRight = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[1]);
    	counter = new Counter(RobotMap.ShooterPivot.HALL_EFFECT_SENSOR); //according to wpi.lib?
    	rightSol = new DoubleSolenoid649(RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[2],
    			RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[3]);
    	leftSol = new DoubleSolenoid649(RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[2],
    			RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[3]);
    	pidLeft = this.getPIDController();
    	pidRight.setOutputRange(PivotPID.MIN_MOTOR_POWER, PivotPID.MAX_MOTOR_POWER);
    	
    	encoderLeft = new Encoder(RobotMap.ShooterPivot.ENCODER1[0], RobotMap.ShooterPivot.ENCODER1[1], false, EncodingType.k2X);
    	encoderRight = new Encoder(RobotMap.ShooterPivot.ENCODER2[0], RobotMap.ShooterPivot.ENCODER2[1], false, EncodingType.k2X);
    	encoderLeft.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    	encoderRight.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    	counter.setReverseDirection(false);
    	counter.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    }
    
    public void resetEncoders() {
    	
    	encoderLeft.reset();
    	encoderRight.reset();
    }
    
    public void resetCounter() {
    	
    	counter.reset();
    }
    
    public boolean reachedLimit() {
    	
    	return counter.get() > 0;
    }
    
    public void runShooter(double power) {
    	
    	motorLeft.set(power);
    	motorRight.set(power);
    }
    
    protected double returnPIDInput() {
    	double dist1 = encoderLeft.getDistance();
    	double dist2 = encoderRight.getDistance();
    	return (dist1 + dist2)/2;
    	
    }
    
    public void setSol(boolean set) {
    	rightSol.set(set);
    	leftSol.set(set);
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	runShooter(output);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
