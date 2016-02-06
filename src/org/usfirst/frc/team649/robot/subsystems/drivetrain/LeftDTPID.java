package org.usfirst.frc.team649.robot.subsystems.drivetrain;


import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LeftDTPID extends PIDSubsystem {

    public PIDController encoderDriveLeftPID;
    
    //PID Constants
    public static final double AUTO_P = 0.0;
    public static final double AUTO_I = 0.0;
    public static final double AUTO_D = 0.0;
    public static final double MAX_MOTOR_POWER = 1.0;
    public static final double MIN_MOTOR_POWER = -1.0;
    
    
    public LeftDTPID() {
    	super("DT Left", AUTO_P, AUTO_I, AUTO_D);

       	
    	encoderDriveLeftPID = this.getPIDController();
    	encoderDriveLeftPID.setAbsoluteTolerance(DrivetrainSubsystem.PIDAbsoluteTolerance);
    	encoderDriveLeftPID.setOutputRange(MAX_MOTOR_POWER, MIN_MOTOR_POWER);
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	protected double returnPIDInput() {
		return Robot.drivetrain.getDistanceDTLeft();
	}

	protected void usePIDOutput(double output) {
        Robot.drivetrain.motors[2].set(output);
        Robot.drivetrain.motors[3].set(output);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

