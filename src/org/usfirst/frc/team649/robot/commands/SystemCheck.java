package org.usfirst.frc.team649.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class SystemCheck extends Command {

	int currentCommandState;
	boolean hasFinishedIntakes, hasFinished;
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		currentCommandState = 1;
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		switch(currentCommandState){
		
		}
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
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
