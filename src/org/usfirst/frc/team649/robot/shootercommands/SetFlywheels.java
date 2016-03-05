package org.usfirst.frc.team649.robot.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetFlywheels extends Command {
	double left,right;
	
    public SetFlywheels(double l, double r) {
    	requires(Robot.shooter);
    	left = l;
    	right  = r;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.setLeftFlywheelPower(left);
    	Robot.shooter.setRightFlywheelPower(right);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	SmartDashboard.putString("SHOOTER Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
