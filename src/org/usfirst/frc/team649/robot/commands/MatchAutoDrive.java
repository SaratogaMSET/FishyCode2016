package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;

public class MatchAutoDrive extends Command {

	public int index;
	public double[][] array;
	
	public MatchAutoDrive(double[][] arr){
		array = arr;
	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		index = 0;
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		
		//HELLA COOL
		try{
			double left = array[index][1] + DrivetrainSubsystem.PIDConstants.k_P * getEncoderErrorLeft(index);
			double right = array[index][2] + DrivetrainSubsystem.PIDConstants.k_P * getEncoderErrorRight(index);
			Robot.drivetrain.rawDrive(left, right);
		}
		catch (Exception e){
			System.out.println("ERROR ERROR ERROR ERROR ERROR ERROR: " + e.getMessage());
		}
		index++;
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return index >= array.length;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.drivetrain.rawDrive(0, 0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}
	
	public double getEncoderErrorLeft(int index){
		return array[index][3] - Robot.drivetrain.getDistanceDTLeft();
	}
	
	public double getEncoderErrorRight(int index){
		return array[index][4] - Robot.drivetrain.getDistanceDTRight();
	}

}
