package org.usfirst.frc.team649.robot.commands;


import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.PIDConstants;

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
public class RightDTPID extends PIDSubsystem {

    public PIDController encoderDriveRightPID;
    
    
    public RightDTPID() {
    	super("DT Right", PIDConstants.k_P, PIDConstants.k_I, PIDConstants.k_D);

       	
    	encoderDriveRightPID = this.getPIDController();
    	encoderDriveRightPID.setAbsoluteTolerance(0.8);
    	//encoderDrivePID.setOutputRange(-EncoderBasedDriving.MAX_MOTOR_POWER, EncoderBasedDriving.MAX_MOTOR_POWER);
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	protected double returnPIDInput() {
		return Robot.drivetrain.getDistanceDTRight();
	}

	protected void usePIDOutput(double output) {
        Robot.drivetrain.motors[2].set(-output);
        Robot.drivetrain.motors[3].set(-output);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
//    public PIDController getGyroPIDControler() {
//    	return encoderTurnPID;
//    }
//	@Override
//	public void pidWrite(double output) {
//        
//        driveFwdRot(0, output);
//	}
//	@Override
//	public double pidGet() {
//		return this.encoders[0].getDistance();
//	}
}

