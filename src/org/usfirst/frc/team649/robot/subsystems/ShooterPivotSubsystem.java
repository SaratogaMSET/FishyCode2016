package org.usfirst.frc.team649.robot.subsystems;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
	public Counter intermediateHalEffect; // unsure about validity of
											// counter/hall effect
	public DigitalInput resetBumperLeft;
	public DigitalInput resetBumperRight;
	public DoubleSolenoid leftSol, rightSol;
	public PowerDistributionPanel pdp = new PowerDistributionPanel();

	public static class PivotPID {

		public static final double ENCODER_DEGREES_PER_PULSE = 360.0 / 256.0
				* 20.0 / 50.0 * 20.0 / 48.0 * 16.0 / 34.0; // change of course
		public static final double k_P = 0.20;
		public static final double k_I = 0.00;
		public static final double k_D = 0.0;
		public static final double ABS_TOLERANCE = 3.0;
		public static final double MAX_MOTOR_UP_POWER = 0.5;
		public static final double MAX_MOTOR_DOWN_POWER = -0.4;
		public static final double ZEROING_CONSTANT_MOVE_POWER = -0.2;

		public static final int PICKUP_STATE = 0;
		public static final int STORING_STATE = 1;
		public static final int SHOOT_STATE = 2;

		public static final double PICKUP_POSITION = 0;
		public static final double STORE_POSITION = 40;// temp value
		public static final double SHOOT_POSITION = 75;// arbitrary value

		public static final double BOTTOM_OF_INTAKE_ZONE = 7;
		public static final double TOP_OF_INTAKE_ZONE = 65;

		public static double MAX_ENCODER_VAL = 90;
		public static double MIN_ENCODER_VAL = 0;

		public static double LEVER_TOLERANCE = 0.03;
		
		public static double CURRENT_LIMIT = 0.75;
		public static double MINIMUM_ENCODER_RATE = 0;
		
		

	}

	public ShooterPivotSubsystem() {

		super("shooter pivot", PivotPID.k_P, PivotPID.k_I, PivotPID.k_D);

		// Init Hardware
		motorLeft = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[0]);
		motorRight = new Victor(RobotMap.ShooterPivot.MOTOR_PORTS[1]);
		rightSol = new DoubleSolenoid(
				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[0],
				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[1],
				RobotMap.ShooterPivot.RIGHT_SOLENOID_PORTS[2]);

		leftSol = new DoubleSolenoid(
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[0],
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[1],
				RobotMap.ShooterPivot.LEFT_SOLENOID_PORTS[2]);

		// Init Sensors
		encoderLeft = new Encoder(RobotMap.ShooterPivot.RIGHT_ENCODER[0],
				RobotMap.ShooterPivot.RIGHT_ENCODER[1], false);
		encoderRight = new Encoder(RobotMap.ShooterPivot.LEFT_ENCODER[0],
				RobotMap.ShooterPivot.LEFT_ENCODER[1], true);

		encoderLeft.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);
		encoderRight.setDistancePerPulse(PivotPID.ENCODER_DEGREES_PER_PULSE);

		intermediateHalEffect = new Counter(
				RobotMap.ShooterPivot.HALL_EFFECT_SENSOR); // according to
															// wpilib?
		resetBumperLeft = new DigitalInput(
				RobotMap.ShooterPivot.RESET_BUMPER_LEFT);
		resetBumperRight = new DigitalInput(
				RobotMap.ShooterPivot.RESET_BUMPER_RIGHT);
		// PID
		pid = this.getPIDController();
		pid.setOutputRange(PivotPID.MAX_MOTOR_DOWN_POWER,
				PivotPID.MAX_MOTOR_UP_POWER);
		pid.setAbsoluteTolerance(PivotPID.ABS_TOLERANCE);
	}

	// //////// LOWER LIMITS
	public boolean lowerLimitsTriggered() {
		return resetBumperLeft.get() || resetBumperRight.get();
	}

	// //////////HALL EFFECTS
	public void updateHalEffect() {
		if (reachedResetLimit()) {
			resetCounter();
		}
	}

	// create a method to tell how fast you are going in Degrees/Second(goes in
	// encoders.

	public void resetCounter() {
		intermediateHalEffect.reset();
	}

	//
	public boolean reachedResetLimit() {
		return intermediateHalEffect.get() > 0;
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
		return (dist1 + dist2) / 2;
	}

	protected double returnPIDInput() {
		return getPivotAngle();
	}

	public void engageBrake(boolean set) {
		if (set) {
			rightSol.set(DoubleSolenoid.Value.kReverse);
			leftSol.set(DoubleSolenoid.Value.kReverse);
		} else {
			rightSol.set(DoubleSolenoid.Value.kForward);
			leftSol.set(DoubleSolenoid.Value.kForward);
		}
	}

	protected void usePIDOutput(double output) {
		if (output > PivotPID.MAX_MOTOR_UP_POWER) {
			output = PivotPID.MAX_MOTOR_DOWN_POWER;
		} else if (output < PivotPID.MAX_MOTOR_DOWN_POWER) {
			output = PivotPID.MAX_MOTOR_DOWN_POWER;
		}

		setPower(output);
	}

	// method that returns the average of the current of the 2 motors
	public double averageMotorCurrents() {
		return ((pdp.getCurrent(RobotMap.ShooterPivot.LEFT_PDP_PORT) + pdp
				.getCurrent(RobotMap.ShooterPivot.RIGHT_PDP_PORT)) / 2);
	}
	public double getRate()
	{
		return ((Robot.shooterPivot.encoderLeft.getRate() + Robot.shooterPivot.encoderRight.getRate())/2);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
