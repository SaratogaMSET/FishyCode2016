package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetPivotPower extends Command {

	double power;
    public SetPivotPower(double pwr) {
        // Use requires() here to declare subsystem dependencies
        power = pwr;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	Robot.shooterPivot.setPower(power);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	SmartDashboard.putString("SHOOTERPIVOT Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    }
}
