package org.usfirst.frc.team649.robot.subsystems.drivetrain;


import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class RightDTPID extends PIDSubsystem {

	 public PIDController encoderDriveRightPID;
	    
	    //PID Constants
	    public static final double AUTO_P = 0.0;
	    public static final double AUTO_I = 0.0;
	    public static final double AUTO_D = 0.0;
	    public static final double MAX_MOTOR_POWER = 1.0;
	    public static final double MIN_MOTOR_POWER = -1.0;
	    
	    
	    public RightDTPID()  {
	    	super("DT Left", AUTO_P, AUTO_I, AUTO_D);

	       	
	    	encoderDriveRightPID = this.getPIDController();
	    	encoderDriveRightPID.setAbsoluteTolerance(DrivetrainSubsystem.PID_ABSOLUTE_TOLERANCE);
	    	encoderDriveRightPID.setOutputRange(MAX_MOTOR_POWER, MIN_MOTOR_POWER);
	        
	    }
	    // Put methods for controlling this subsystem
	    // here. Call these from Commands.

		protected double returnPIDInput() {
			return Robot.drivetrain.getDistanceDTLeft();
		}

		protected void usePIDOutput(double output) {
	        Robot.drivetrain.motors[0].set(output);
	        Robot.drivetrain.motors[1].set(output);
		}

	    public void initDefaultCommand() {
	        // Set the default command for a subsystem here.
	        //setDefaultCommand(new MySpecialCommand());
	    }
}

