package org.usfirst.frc.team649.robot.subsystems.drivetrain;


import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class LeftDTPID extends PIDSubsystem {

    public PIDController encoderDriveLeftPID;
    
    
    public LeftDTPID() {
    	super("DT Left", DrivetrainSubsystem.PIDConstants.k_P, DrivetrainSubsystem.PIDConstants.k_I, DrivetrainSubsystem.PIDConstants.k_D);

       	
    	encoderDriveLeftPID = this.getPIDController();
    	encoderDriveLeftPID.setAbsoluteTolerance(0.8);
    	//encoderDrivePID.setOutputRange(-EncoderBasedDriving.MAX_MOTOR_POWER, EncoderBasedDriving.MAX_MOTOR_POWER);
        
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