package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class EngageBrakes extends Command {
	Timer t;
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		Robot.shooterPivot.engageBrake(true);
		t = new Timer();
		t.start();
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub

	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return t.get() > 0.15;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.shooterPivot.setPower(0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}

}
