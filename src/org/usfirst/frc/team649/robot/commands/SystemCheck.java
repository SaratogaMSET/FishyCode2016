package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.OI;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SystemCheck extends Command {

	int currentCommandState;
	boolean hasFinishedIntakes, hasFinished;
	boolean prevStateTrigger;
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		currentCommandState = 0;
		prevStateTrigger = false;
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		switch(currentCommandState){
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			default:
				break;
		}
		
		if (Robot.oi.operatorJoystick.getRawButton(1) && !prevStateTrigger){
			currentCommandState++;
		}
		
		prevStateTrigger = Robot.oi.operatorJoystick.getRawButton(1);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return currentCommandState == 8;
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
