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
	public Counter intermediateHalEffect; //unsure about validity of counter/hall effect
	public DigitalInput resetBumperLeft;
	public DigitalInput resetBumperRight;
	public DoubleSolenoid leftSol, rightSol;
	
	public static class PivotPID {
		
		public static final double ENCODER_DEGREES_PER_PULSE = 360.0/256.0 * 20.0/50.0 * 20.0/48.0 * 16.0/34.0; //change of course
		public static final double k_P = 0.10;
		public static final double k_I = 0.00;
		public static final double k_D = 0.0;
		public static final double ABS_TOLERANCE = 3;
		public static final double MAX_MOTOR_UP_POWER = 0.5;
		public static final double MAX_MOTOR_DOWN_POWER = -0.2;
		public static final double ZEROING_CONSTANT_MOVE_POWER = 0;


		public static final int PICKUP_STATE = 0;
		public static final int STORING_STATE = 1;
		public static final int SHOOT_STATE = 2;
		
		public static final double PICKUP_POSITION = 0;//arbitrary value
		public static final double STORE_POSITION = 10;//arbitrary value
		public static final double SHOOT_POSITION = 75;//arbitrary value
		
		public static final double BOTTOM_OF_INTAKE_ZONE = 20;
		public static final double TOP_OF_INTAKE_ZONE = 55;
		
		public static double MAX_ENCODER_VAL = 90;
		public static double MIN_ENCODER_VAL = 10;
		
		public static double LEVER_TOLERANCE = 0.03;
		
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
    			RobotMap.ShooterPivot.RIGHT_ENCODER[1], false);
    	encoderRight = new Encoder(RobotMap.ShooterPivot.LEFT_ENCODER[0], 
    			RobotMap.ShooterPivot.LEFT_ENCODER[1], true);
    
    	encoderLeft.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
    	encoderRight.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
    	
    	intermediateHalEffect = new Counter(RobotMap.ShooterPivot.HALL_EFFECT_SENSOR); //according to wpilib?
    	resetBumperLeft = new DigitalInput(RobotMap.ShooterPivot.RESET_BUMPER_LEFT);
    	resetBumperRight = new DigitalInput(RobotMap.ShooterPivot.RESET_BUMPER_RIGHT);
    	//PID 
    	pid = this.getPIDController();
    	pid.setOutputRange(PivotPID.MAX_MOTOR_DOWN_POWER, PivotPID.MAX_MOTOR_UP_POWER);
    	pid.setAbsoluteTolerance(PivotPID.ABS_TOLERANCE);
    	
    }
    ////////// LOWER LIMITS
    public boolean lowerLimitsTriggered(){
    	return resetBumperLeft.get() || resetBumperRight.get();
    }
    
    ////////////HAL EFFECTS
    public void updateHalEffect(){
    	if (reachedResetLimit()){
    		resetCounter();
    	}
    }
    
    public void resetCounter() {
    	intermediateHalEffect.reset();
    }
//    
    public boolean reachedResetLimit() {
    	return intermediateHalEffect.get() > 0;
    }
    
    /////////////ENCODERS
    public boolean pastMax(){
		return encoderLeft.getDistance() > PivotPID.MAX_ENCODER_VAL;
	}
    
    public boolean isBelowIntakeZone() {
    	return getPosition() < PivotPID.BOTTOM_OF_INTAKE_ZONE;
    }
    
    public boolean isAboveIntakeZone() {
    	return getPosition() > PivotPID.TOP_OF_INTAKE_ZONE;
    }
    public void resetEncoders() {
    	encoderLeft.reset();
    	encoderRight.reset();
    }
    
    public void setPower(double power) {
    
    	motorLeft.set(power);
    	SmartDashboard.putNumber(" motor power", power);
    	motorRight.set(-power);
    }
    
    public double getPivotAngle() {
    	double dist1 = encoderLeft.getDistance();
    	double dist2 = encoderRight.getDistance();
    	return dist1;//(dist1 + dist2)/2;
    }
    protected double returnPIDInput() {
    	return getPivotAngle();
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
    	if (output > PivotPID.MAX_MOTOR_UP_POWER){
    		output = PivotPID.MAX_MOTOR_DOWN_POWER;
    	}
    	else if (output < PivotPID.MAX_MOTOR_DOWN_POWER){
    		output = PivotPID.MAX_MOTOR_DOWN_POWER;
    	}
    	
    	setPower(output);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}
