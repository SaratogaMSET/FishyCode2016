package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
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
	public DigitalInput resetHalEffectLeft; 
	public DigitalInput resetHalEffectRight;
									// unsure about validity of
											// counter/hall effect
	public DigitalInput resetBumperLeft;
	public DigitalInput resetBumperRight;
	public DoubleSolenoid brake;//, rightSol;
	
	public int currentPivotState;

	public static class PivotPID {

		public static final double ENCODER_DEGREES_PER_PULSE = 360.0 / 256.0
				* 20.0 / 50.0 * 20.0 / 48.0 * 16.0 / 48.0; 
		public static final double k_P = 0.12;
		public static final double k_I = 0.000;
		public static final double k_D = 0.055;
		public static final double ABS_TOLERANCE = .30;
		
		public static double max_motor_up_power = 0.6; //changed in pivot state command
		public static final double MIDDLE_STATE_MAX_UP_POWER = 0.3;
		public static final double REGULAR_MAX_UP_POWER = 0.6;
		public static final double MAX_MOTOR_DOWN_POWER = -0.4;
		public static final double MIN_PIVOT_SPEED = 0;
		public static final double ZEROING_CONSTANT_MOVE_POWER = -0.3;
		
		
		public static final int PICKUP_STATE = 0;
		public static final int STORING_STATE = 1;
		public static final int FAR_SHOOT_STATE = 2;
		public static final int CLOSE_SHOOT_STATE = 3;
		public static final int BACK_SHOOT_STATE = 4;
		public static final int CURRENT_STATE = 5;
		

		public static final double PICKUP_POSITION = 1.0;
		public static final double STORE_POSITION = 8.0;// temp value
		public static final double FAR_SHOOT_POSITION = 43.0; //44.7
		
		public static final double CLOSE_SHOOT_POSITION = 72.4;//68.3;//62.2;
		public static final double BACK_SHOOT_POSITION = 115.0;//62.2;
		
		public static final double AUTO_POS1_SHOOT_POSITION = 45.0;
		public static final double AUTO_POS2_SHOOT_POSITION = 42.0;
		public static final double AUTO_POS3_SHOOT_POSITION = 37.0;
		public static final double AUTO_POS4_SHOOT_POSITION = 42.0;
		public static final double AUTO_POS5_SHOOT_POSITION = 37.0;
		public static final double AUTO_CAMERA_AIM_POSITION = 32.0;
		
		public static final double REGION_ERROR = 4.0; //degrees

		public static final double BOTTOM_OF_INTAKE_ZONE = 9.0;
		public static final double TOP_OF_INTAKE_ZONE = 60;
		public static final double MIDDLE_OF_INTAKE_ZONE = 35;
		public static final double HOLD_PIVOT_POSITION_POWER = 0.04;

		public static double MAX_ENCODER_VAL = 130;
		public static double MIN_ENCODER_VAL = 0;

		public static double LEVER_TOLERANCE = 0.03;
		
		public static double CURRENT_LIMIT = 1.0;
		public static double MINIMUM_ENCODER_RATE = 0;
		
		

	}

	public ShooterPivotSubsystem() {

		super("shooter pivot", PivotPID.k_P, PivotPID.k_I, PivotPID.k_D);

		// Init Hardware
		motorLeft = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[0]);
		motorRight = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[1]);
//		rightSol = new DoubleSolenoid(
//				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[0],
//				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[1],
//				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[2]);

		brake = new DoubleSolenoid(
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[0],
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[1],
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[2]);

		// Init Sensors
		encoderLeft = new Encoder(RobotMap.ShooterPivot.LEFT_ENCODER[0],
				RobotMap.ShooterPivot.LEFT_ENCODER[1], false);
		encoderRight = new Encoder(RobotMap.ShooterPivot.RIGHT_ENCODER[0],
				RobotMap.ShooterPivot.RIGHT_ENCODER[1], true);

		encoderLeft.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
		encoderRight.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);

		resetHalEffectRight = new DigitalInput(
				RobotMap.ShooterPivot.RESET_HAL_EFFECT_RIGHT); // according to
		resetHalEffectLeft = new DigitalInput(RobotMap.ShooterPivot.RESET_HAL_EFFECT_LEFT);												// wpilib?
		resetBumperLeft = new DigitalInput(
				RobotMap.ShooterPivot.RESET_BUMPER_LEFT);
		resetBumperRight = new DigitalInput(
				RobotMap.ShooterPivot.RESET_BUMPER_RIGHT);
		
		// PID
		pid = this.getPIDController();
		pid.setOutputRange(PivotPID.MAX_MOTOR_DOWN_POWER,
				PivotPID.max_motor_up_power);
		pid.setAbsoluteTolerance(PivotPID.ABS_TOLERANCE);
		
		
		currentPivotState = -1; //always at beginning of match
	}

	// //////// LOWER LIMITS
	public boolean lowerLimitsTriggered() {
		return !resetBumperLeft.get() || !resetBumperRight.get();
	}

	// //////////HALL EFFECTS
//	public void updateHalEffect() {
//		if (reachedResetLimit()) {
//			resetCounter();
//		}
//	}

	// create a method to tell how fast you are going in Degrees/Second(goes in
	// encoders.

//	public void resetCounter() {
//		intermediateHalEffect.reset();
//	}
//
//	//
//	public boolean reachedResetLimit() {
//		return intermediateHalEffect.get() > 0;
//	}

	public int getPivotState(){
		return currentPivotState;
	}
	
	// ///////////ENCODERS
	public boolean pastMax() {
		return encoderLeft.getDistance() > PivotPID.MAX_ENCODER_VAL;
	}

	public boolean isBelowIntakeZone() {
		return getPosition() < PivotPID.BOTTOM_OF_INTAKE_ZONE;
	}

	public boolean isAboveIntakeZone() {
		return getPosition() > PivotPID.TOP_OF_INTAKE_ZONE;
	}
	
	public boolean isInIntakeZone() {
		return !isBelowIntakeZone() && !isAboveIntakeZone();
	}
	
	public boolean isResetHalTripped(){
		return !resetHalEffectLeft.get() || !resetHalEffectRight.get();
	}
	
	
	public void resetEncoders() {
		encoderLeft.reset();
		encoderRight.reset();
	}

	public void setPower(double power) {

		motorLeft.set(power);
		SmartDashboard.putNumber(" motor power", power);
		motorRight.set(-power/0.94);
	}

	public double getPivotAngle() {
		double diffEncoders = encoderLeft.getDistance()- encoderRight.getDistance();
		if (Math.abs(diffEncoders) < 5){
			return (encoderLeft.getDistance() + encoderRight.getDistance()) / 2.0;
		}
		else { 
			if (encoderLeft.getDistance() - encoderRight.getDistance()> 0){
				return encoderLeft.getDistance();
			}
			else{
				return encoderRight.getDistance();
			}
		}
			//		double dist2 = encoderRight.getDistance();
//		return (dist1 + dist2) / 2;
	}
	
	public double getClosestAngleToSetpoint(double setpoint){
		double diffL = Math.abs(encoderLeft.getDistance() - setpoint);
		double diffR = Math.abs(encoderRight.getDistance() - setpoint);
		double diffEncoders = encoderLeft.getDistance()- encoderRight.getDistance();
		
		if (Math.abs(diffEncoders) < 8){
			if (diffL <= diffR){
				return encoderLeft.getDistance();
			}
			else {
				return encoderRight.getDistance();
			}
		}
		else{
			if (diffEncoders > 0){
				//left is greater than right
				return encoderLeft.getDistance();
			}
			else{
				return encoderRight.getDistance();
			}
		}
	}
	
	//there are 5 regions: RESET, STORE, CLOSE_SHOT, FAR_SHOT, and BACK_SHOT
	//this function returns the closest NEXT state to go to based on where we are (with tolerance)
	public int getClosestNextSetpointState(boolean up){
		double pos = getPosition();
		//if traveling up
		if (up){ 
			if (pos < PivotPID.STORE_POSITION - PivotPID.REGION_ERROR){ //IN THE LOWEST REGION, so go UP to store
				return PivotPID.STORING_STATE;
			}
			else if (pos < PivotPID.FAR_SHOOT_POSITION - PivotPID.REGION_ERROR){ //IN THE REGION OF STORE, so go UP to CLOSE SHOOT
				return PivotPID.FAR_SHOOT_STATE;
			}
			else if (pos < PivotPID.CLOSE_SHOOT_POSITION - PivotPID.REGION_ERROR){ //IN THE REGION OF CLOSE SHOOT, so go UP to FAR SHOOT
				return PivotPID.CLOSE_SHOOT_STATE;
			}
			else{ //IN ANY OTHER CASE, JUST GO UP TO BACK SHOOT
				return PivotPID.BACK_SHOOT_STATE;
			}
		}
		//if traveling down
		else{ 
			if (pos > PivotPID.BACK_SHOOT_POSITION + PivotPID.REGION_ERROR){ //IN THE LOWEST REGION, so go UP to store
				return PivotPID.BACK_SHOOT_STATE;
			}
			else if (pos > PivotPID.CLOSE_SHOOT_POSITION + PivotPID.REGION_ERROR){ //IN THE REGION OF STORE, so go UP to CLOSE SHOOT
				return PivotPID.CLOSE_SHOOT_STATE;
			}
			else if (pos > PivotPID.FAR_SHOOT_POSITION + PivotPID.REGION_ERROR){ //IN THE REGION OF CLOSE SHOOT, so go UP to FAR SHOOT
				return PivotPID.FAR_SHOOT_STATE;
			}
			else if (pos > PivotPID.STORE_POSITION + PivotPID.REGION_ERROR){ //IN THE LOWEST REGION, so go UP to store
				return PivotPID.STORING_STATE;
			}
			else{ //IN ANY OTHER CASE, JUST GO RESET
				return PivotPID.PICKUP_STATE;
			}
		}
	}

	@Override
	public double returnPIDInput() {
		//return getPivotAngle();
		return getClosestAngleToSetpoint(this.pid.getSetpoint());
	}

	public void engageBrake(boolean set) {
		if (set) {
			//rightSol.set(DoubleSolenoid.Value.kReverse);
			setPower(0);
			brake.set(DoubleSolenoid.Value.kReverse);
		} else {
			//rightSol.set(DoubleSolenoid.Value.kForward);
			brake.set(DoubleSolenoid.Value.kForward);
		}
	}

	@Override
	protected void usePIDOutput(double output) {
		if (output > PivotPID.max_motor_up_power) {
			output = PivotPID.max_motor_up_power;
		} else if (output < PivotPID.MAX_MOTOR_DOWN_POWER) {
			output = PivotPID.MAX_MOTOR_DOWN_POWER;
		}
		double powerUpOrDown = 1.0;
		if(getPivotAngle() > 80) {
		//	powerUpOrDown = -1.0;
		}
		setPower(output + powerUpOrDown * PivotPID.HOLD_PIVOT_POSITION_POWER /*account for gravety*/);
	}

	// method that returns the average of the current of the 2 motors
	public double averageMotorCurrents() {
		return ((Robot.pdp.getCurrent(RobotMap.ShooterPivot.LEFT_PDP_PORT) + Robot.pdp.getCurrent(RobotMap.ShooterPivot.RIGHT_PDP_PORT)) / 2);
	}
	
	public double getEncoderRate()
	{
		return ((Robot.shooterPivot.encoderLeft.getRate() + Robot.shooterPivot.encoderRight.getRate())/2);
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public boolean isReadyToPickUp() {
		// TODO Auto-generated method stub
		return getPosition() <= PivotPID.PICKUP_POSITION;
	}
	
	public boolean isOnTarget(double setpoint){
		return Math.abs(getClosestAngleToSetpoint(setpoint) - setpoint) < PivotPID.ABS_TOLERANCE;
	}
}
