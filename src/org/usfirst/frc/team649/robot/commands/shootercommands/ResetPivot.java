package org.usfirst.frc.team649.robot.commands.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ResetPivot extends Command {
	
    public ResetPivot() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("ACTIVE COMMAND", "Run Til Hall Effect");
    	Robot.shooterPivotSubsystem.runShooter(ShooterPivotSubsystem.PivotPID.SHOOTER_POWER);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.shooterPivotSubsystem.reachedLimit();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooterPivotSubsystem.resetCounter();
    	Robot.shooterPivotSubsystem.resetEncoders();
    	Robot.shooterPivotSubsystem.runShooter(0);
    	Robot.shooterPivotSubsystem.setSol(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
