package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MatchAutoDrive extends Command {

	public int index, pos;
	public double diff;
	public double[][] array;
	
	public MatchAutoDrive(double[][] arr, int pos){
		array = arr;
		this.pos = pos;
		
		//calculate difference with opencv based on pos
		//just a thought: Robot.cameraSubystem.calculatePictureDifference();
		diff = 0;
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
			double left = array[index][1];// + DrivetrainSubsystem.PIDConstants.k_P * getEncoderErrorLeft(index) + AutonomousSequences.visionOffset(pos, diff, index)[0];
			double right = array[index][2];// + DrivetrainSubsystem.PIDConstants.k_P * getEncoderErrorRight(index) + AutonomousSequences.visionOffset(pos, diff, index)[1];
			Robot.drivetrain.rawDrive(-right, left);
		}
		catch (Exception e){
			System.out.println("ERROR ERROR ERROR ERROR ERROR ERROR: " + e.getMessage());
		}
		index++;
		
		

    	SmartDashboard.putString("DT Current Command", this.getName());
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return index >= array.length
				|| Robot.oi.driver.isManualOverride();
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
