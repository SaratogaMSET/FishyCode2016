package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;

import edu.wpi.first.wpilibj.command.Command;

public class TurnWithGyro extends Command {
	public double angle;
	//positive is clockwise
	public TurnWithGyro(double angle) {
		// TODO Auto-generated constructor stub
		this.angle = angle;
	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		double error = 0;//angle - Robot.drivetrain.gyro.getAngle();
		Robot.drivetrain.rawDrive(error * TurnConstants.P_VAL, -error * TurnConstants.P_VAL);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false
				|| Robot.oi.driver.isManualOverride(); //Math.abs(angle - Robot.drivetrain.gyro.getAngle()) < TurnConstants.TOLERANCE;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.drivetrain.driveFwdRot(0, 0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}

}
