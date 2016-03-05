package org.usfirst.frc.team649.robot.subsystems.drivetrain;

import java.util.ArrayList;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainSubsystem extends PIDSubsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public CANTalon [] motors;
	public Encoder leftEncoder, rightEncoder;
	
	public BuiltInAccelerometer accel;
	public DoubleSolenoid driveSol;
	public Gyro gyro;//ADXRS450_Gyro gyro;
	
	public Compressor compressor;
	
	public static final double highGearEncoderDistancePerPulse = 18.85  * 24.0/50.0 / 256;
	public static final double lowGearEncoderDistancePerPulse = 18.85 * 14.0/60.0 / 256;
	
	public static final double powerRatio = 0.97; //right : left (right motor has less power on pbot) //TODO TUNE
	public PIDController encoderDrivePID;
	
	public static boolean HIGH_GEAR = true;
	
	
	public static class PIDConstants {
		public static final double PID_ABSOLUTE_TOLERANCE =1.0;
		public static  double k_P = .02; //0.2
		public static double k_I = 0.0001;
		public static double k_D = 0.03;
	}
	
	public static class TurnConstants { 
		public static final double P_VAL = .40;
		public static final double I_VAL = 0;
		public static final double D_VAL = 0.0;
		
		public static final double TOLERANCE = 0.8; //degrees
	}
	
	public static class AutoConstants {
		public static final int LOW_BAR = 0;
		public static final int PORTCULLIS = 1;
		public static final int CHEVAL_DE_FRISE = 2;
		public static final int ROCK_WALL = 3;
		public static final int ROUGH_TERRAIN = 4;
		public static final int SALLY_PORT = 5;
		public static final int DRAWBRIDGE = 6;
		public static final int RAMPARTS = 7;
		public static final int MOAT = 8;
		
		
		//CHEVAL CONSTANTS
		public static final double DISTANCE_START_TO_RAMP_CHEVAL = 68.0;
		public static final double DISTANCE_RAMP_TO_MIDRAMP_CHEVAL = 20.0;
		public static final double WAIT_TIME_AT_TOP_CHEVAL = 0.75;
		public static final double DISTANCE_OFF_CHEVAL = 18.0;
		
		//ROCKWALL CONSTANTS
		public static final double DISTANCE_OFF_ROCKWALL = 18;
		
		
		//MOAT CONSTANTS
		public static final double DISTANCE_START_TO_RAMP_MOAT = 18;
		public static final double DISTANCE_RAMP_TO_END_MOAT = 18;
		
		//PORTCULLIS CONSTANTS
		
		public static final double ACCEL_CHANGE_THRESHOLD = 0;
		public static final double DISTANCE_START_TO_RAMP_PORTCULLIS = 44;
		public static final double DISTANCE_RAMP_TO_END_PORTCULLIS = 21;
	}
	
	
	
	public DrivetrainSubsystem() {
		super ("Drivetrain", PIDConstants.k_P, PIDConstants.k_I, PIDConstants.k_D);
		motors = new CANTalon[4];
		gyro = new AnalogGyro(0);//new ADXRS450_Gyro();
		//compressor = new Compressor();
		gyro.reset();
		accel = new BuiltInAccelerometer();
		//FR,BR,BL,BR																																																																																																																																																																																																																																																																																																																						
		for(int i =0; i < motors.length; i++) {
			motors[i] = new CANTalon(RobotMap.Drivetrain.MOTOR_PORTS[i]);
		}
		leftEncoder = new Encoder(RobotMap.Drivetrain.LEFT_ENCODER_FORWARD_CHANNEL, 
				RobotMap.Drivetrain.LEFT_ENCODER_REVERSE_CHANNEL, false);
		leftEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
		
		rightEncoder = new Encoder(RobotMap.Drivetrain.RIGHT_ENCODER_FORWARD_CHANNEL, 
				RobotMap.Drivetrain.RIGHT_ENCODER_REVERSE_CHANNEL, true);
		leftEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
		
		driveSol  = new DoubleSolenoid(RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[0],
				RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[1], RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[2]);
		
		leftEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
		rightEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
		
		encoderDrivePID = this.getPIDController();
		encoderDrivePID.setAbsoluteTolerance(PIDConstants.PID_ABSOLUTE_TOLERANCE);
		encoderDrivePID.setOutputRange(-.2, .5);
		
	}

	public void driveFwdRot(double fwd, double rot) {
		double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }

    public void rawDrive(double left, double right) {
        motors[0].set(powerRatio* right);
        motors[1].set(powerRatio * right);
        motors[2].set(-left);
        motors[3].set(-left);
        
        SmartDashboard.putNumber("DriveMotorLeft", left);
        SmartDashboard.putNumber("DriveMotorRight", right);
    }
    
    public double getDistanceDTLeft() {
        return leftEncoder.getDistance();
    }

    public double getDistanceDTRight() {
      return rightEncoder.getDistance();
  }
   public double getDistanceDTBoth(){
	   return rightEncoder.getDistance()/2 + leftEncoder.getDistance()/2;
   }
    public ArrayList<Double> getLoggingData() {
    	ArrayList<Double> temp = new ArrayList<Double>();
   	   temp.add(Robot.timer.get());
   	   temp.add(motors[0].get());//right
   	   temp.add(motors[2].get());//left
   	   temp.add(getDistanceDTLeft());
   	   temp.add(getDistanceDTRight());
   	   //temp.add(gyro.getAngle());
   	   
   	   return temp;
   	 
    }
    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }
    
    //true = high gear, false = low gear
    public void shift(boolean highGear) {
    	driveSol.set(highGear ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    	if (highGear){
    		leftEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
    		rightEncoder.setDistancePerPulse(highGearEncoderDistancePerPulse);
    	}
    	else{
    		leftEncoder.setDistancePerPulse(lowGearEncoderDistancePerPulse);
    		rightEncoder.setDistancePerPulse(lowGearEncoderDistancePerPulse);
    	}
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public boolean seesAlignmentLine() {
    	return false;
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return leftEncoder.getDistance();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		driveFwdRot(-output, 0);
		
	}
}

