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
	
	public Victor motor1;
	public Victor motor2;
	public Encoder encoder1;
	public Encoder encoder2;
	public PIDController pid;
	public Counter counter; //unsure about validity of counter/hall effect
	public DoubleSolenoid649 leftSol;
	public DoubleSolenoid649 rightSol;
	
	public static class PivotPID {
		
		private static final double ENCODER_DISTANCE_PER_PULSE = 0; //change of course
		public static final double SHOOTER_P = 0.15;
		public static final double SHOOTER_I = 0.15;
		public static final double SHOOTER_D = 0.15;
		public static final double ABS_TOLERANCE = 0.05;
		public static double MAX_MOTOR_POWER = 0.5;
		public static double MIN_MOTOR_POWER = 0.15;
		public static double SHOOTER_POWER = 0.1; //idk
		
	}


    public ShooterPivotSubsystem() {

    	super("shooter pivot", PivotPID.SHOOTER_P, PivotPID.SHOOTER_I, PivotPID.SHOOTER_D);
    	motor1 = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[0]);
    	motor2 = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[1]);
    	counter = new Counter(RobotMap.ShooterPivot.HALL_EFFECT_SENSOR); //according to wpi.lib?
    	rightSol = new DoubleSolenoid649(RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[2],
    			RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[3]);
    	leftSol = new DoubleSolenoid649(RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[2],
    			RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[3]);
    	pid = this.getPIDController();
    	pid.setOutputRange(PivotPID.MIN_MOTOR_POWER, PivotPID.MAX_MOTOR_POWER);
    	
    	encoder1 = new Encoder(RobotMap.ShooterPivot.ENCODER1[0], RobotMap.ShooterPivot.ENCODER1[1], false, EncodingType.k2X);
    	encoder2 = new Encoder(RobotMap.ShooterPivot.ENCODER2[0], RobotMap.ShooterPivot.ENCODER2[1], false, EncodingType.k2X);
    	encoder1.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    	encoder2.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    	counter.setReverseDirection(false);
    	counter.setDistancePerPulse(PivotPID.ENCODER_DISTANCE_PER_PULSE);
    }
    
    public void resetEncoders() {
    	
    	encoder1.reset();
    	encoder2.reset();
    }
    
    public void resetCounter() {
    	
    	counter.reset();
    }
    
    public boolean reachedLimit() {
    	
    	return counter.get() > 0;
    }
    
    public void runShooter(double power) {
    	
    	motor1.set(power);
    	motor2.set(power);
    }
    
    protected double returnPIDInput() {
    	double dist1 = encoder1.getDistance();
    	double dist2 = encoder2.getDistance();
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
