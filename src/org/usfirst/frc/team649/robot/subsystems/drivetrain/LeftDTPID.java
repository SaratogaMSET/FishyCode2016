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
//    	super("DT Left", DrivetrainSubsystem.PIDConstants.k_P, DrivetrainSubsystem.PIDConstants.k_I, DrivetrainSubsystem.PIDConstants.k_D);

    	super("DT Left", .03, 0, 2);
       	
    	encoderDriveLeftPID = this.getPIDController();
    	encoderDriveLeftPID.setAbsoluteTolerance(0.8);
    	
    	encoderDriveLeftPID.setOutputRange(-0.4, 0.4);
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	protected double returnPIDInput() {
		return Robot.drivetrain.getDistanceDTLeft();
	}

	@Override
	protected void usePIDOutput(double output) {
        Robot.drivetrain.motors[2].set(-output);
        Robot.drivetrain.motors[3].set(-output);
	}

    @Override
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