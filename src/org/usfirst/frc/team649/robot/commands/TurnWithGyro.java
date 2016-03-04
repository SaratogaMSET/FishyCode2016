package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;

import edu.wpi.first.wpilibj.command.Command;

public class TurnWithGyro extends Command {
	public double angle;
	public double setpoint;
	//positive is clockwise
	public TurnWithGyro(double angle) {
		// TODO Auto-generated constructor stub
		this.angle = angle;
	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		setpoint = Robot.drivetrain.gyro.getAngle() + angle;
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		double error = angle - Robot.drivetrain.gyro.getAngle();
		Robot.drivetrain.rawDrive(-error * TurnConstants.P_VAL, error * TurnConstants.P_VAL);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Math.abs(angle - Robot.drivetrain.gyro.getAngle()) < TurnConstants.TOLERANCE
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
