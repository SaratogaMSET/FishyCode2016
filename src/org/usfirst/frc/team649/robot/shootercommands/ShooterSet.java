package org.usfirst.frc.team649.robot.shootercommands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class ShooterSet extends Command {
	
	DoubleSolenoid.Value stateToSet;
	
	public ShooterSet(DoubleSolenoid.Value val){

		stateToSet = val;
	}
	
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		Robot.shooter.loader.set(stateToSet);
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}

}
