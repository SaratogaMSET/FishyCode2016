package org.usfirst.frc.team649.robot.subsystems.drivetrain;

import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.DoubleSolenoid649;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainSubsystem extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	Victor [] motors;
	Encoder leftEncoder, rightEncoder;
	
	public DoubleSolenoid649 driveSol;
	
	public static final double highGearEncoderDistancePerPulse = 34;
	public static final double lowGearEncoderDistancePerPulse = 24;
	
	public static final double PIDAbsoluteTolerance = 0.8;
	
	
	
	public DrivetrainSubsystem() {
		motors = new Victor[4];
		//FR,BR,BL,BR
		for(int i =0; i < motors.length; i++) {
			motors[i] = new Victor(RobotMap.Drivetrain.MOTOR_PORTS[i]);
		}
		leftEncoder = new Encoder(RobotMap.Drivetrain.LEFT_ENCODER_FORWARD_CHANNEL, 
				RobotMap.Drivetrain.LEFT_ENCODER_REVERSE_CHANNEL, true);
		rightEncoder = new Encoder(RobotMap.Drivetrain.RIGHT_ENCODER_FORWARD_CHANNEL, 
				RobotMap.Drivetrain.RIGHT_ENCODER_REVERSE_CHANNEL, true);
		
		driveSol  = new DoubleSolenoid649(RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[0],
				RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[1], RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[3],
				RobotMap.Drivetrain.DRIVE_SOLENOID_PORTS[3]);
	}

	public void driveFwdRot(double fwd, double rot) {
		double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }

    public void rawDrive(double left, double right) {
        motors[0].set(right);
        motors[1].set(right);
        motors[2].set(left);
        motors[3].set(left);
        
        SmartDashboard.putNumber("DriveMotorLeft", left);
        SmartDashboard.putNumber("DriveMotorRight", right);
    }
    
    public double getDistanceDTLeft() {
        return leftEncoder.getDistance();
    }

    public double getDistanceDTRight() {
      return rightEncoder.getDistance();
  }
    
    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }
    public void shift(boolean lowSpeed) {
    	driveSol.set(lowSpeed);
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

