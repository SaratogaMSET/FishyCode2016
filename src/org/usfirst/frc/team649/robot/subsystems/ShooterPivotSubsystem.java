package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterPivotSubsystem extends PIDSubsystem {
	
	public Victor motorLeft, motorRight;
	public Encoder encoderLeft, encoderRight;
	public PIDController pid;
	public Counter resetHalEffect; //unsure about validity of counter/hall effect
	public DoubleSolenoid leftSol, rightSol;
	
	public static class PivotPID {
		
		public static final double ENCODER_DEGREES_PER_PULSE = 360.0/256.0 * 20.0/50.0 * 20.0/48.0; //change of course
		public static final double k_P = 0.10;
		public static final double k_I = 0.00;
		public static final double k_D = 0.0;
		public static final double ABS_TOLERANCE = 3;
		public static final double MAX_MOTOR_POWER = 0.5;
		public static final double MIN_MOTOR_POWER = -0.5;
		public static final double ZEROING_CONSTANT_MOVE_POWER = 0;


		public static final int PICKUP_STATE = 0;
		public static final int STORING_STATE = 1;
		public static final int SHOOT_STATE = 2;
		
		public static final double PICKUP_POSITION = 20;//arbitrary value
		public static final double STORE_POSITION = 90;//arbitrary value
		public static final double SHOOT_POSITION = 150;//arbitrary value
		
	}


    public ShooterPivotSubsystem() {

    	super("shooter pivot", PivotPID.k_P, PivotPID.k_I, PivotPID.k_D);
    	
    	//Init Hardware
    	motorLeft = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[0]);
    	motorRight = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[1]);
    	rightSol = new DoubleSolenoid(RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[2]);
    	
    	leftSol = new DoubleSolenoid(RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[0],
    			RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[1],RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[2]);
    	
    	//Init Sensors
    	encoderLeft = new Encoder(RobotMap.ShooterPivot.RIGHT_ENCODER[0], 
    			RobotMap.ShooterPivot.RIGHT_ENCODER[1], true);
    	encoderRight = new Encoder(RobotMap.ShooterPivot.LEFT_ENCODER[0], 
    			RobotMap.ShooterPivot.LEFT_ENCODER[1], false);
    
    	encoderLeft.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
    	encoderRight.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
    	
    	resetHalEffect = new Counter(RobotMap.ShooterPivot.HALL_EFFECT_SENSOR); //according to wpi.lib?
    	
    	//PID 
    	pid = this.getPIDController();
    	pid.setOutputRange(PivotPID.MIN_MOTOR_POWER, PivotPID.MAX_MOTOR_POWER);
    	pid.setAbsoluteTolerance(PivotPID.ABS_TOLERANCE);
    	
    }
    
    public void resetEncoders() {
    	encoderLeft.reset();
    	encoderRight.reset();
    }
    
    public void resetCounter() {
    	resetHalEffect.reset();
    }
    
    public boolean reachedResetLimit() {
    	return resetHalEffect.get() > 0;
    }
    
    public void setPower(double power) {
    
    	motorLeft.set(-power);
    	SmartDashboard.putNumber(" motor power", power);
    	motorRight.set(power);
    }
    
    protected double returnPIDInput() {
    	double dist1 = encoderLeft.getDistance();
    	double dist2 = encoderRight.getDistance();
    	return dist1;//(dist1 + dist2)/2;
    	
    }
    
    public void engageBrake(boolean set) {
    	if(set) {
    		rightSol.set(DoubleSolenoid.Value.kForward);
    		leftSol.set(DoubleSolenoid.Value.kForward);
    	} else {
    		rightSol.set(DoubleSolenoid.Value.kReverse);
    		leftSol.set(DoubleSolenoid.Value.kReverse);
    	}
    }
    
    protected void usePIDOutput(double output) {
    	setPower(output);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
