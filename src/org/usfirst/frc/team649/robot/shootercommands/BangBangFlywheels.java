package org.usfirst.frc.team649.robot.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BangBangFlywheels extends Command {

	boolean running;
    public BangBangFlywheels(boolean on) {
    	requires(Robot.shooter);
    	running = on;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (running){
 			if(Robot.shooter.getLeftFlywheelRPM() <= ShooterSubsystem.FLYWHEEL_TARGET_RPM){
 				Robot.shooter.setLeftFlywheelPower(ShooterSubsystem.FLYWHEEL_MAX_SHOOT_POWER);
 			}	
 			else{
 				Robot.shooter.setLeftFlywheelPower(ShooterSubsystem.FLYWHEEL_MIN_SHOOT_POWER);
 			}
 			
 			if(Robot.shooter.getRightFlywheelRPM() <= ShooterSubsystem.FLYWHEEL_TARGET_RPM){
 				Robot.shooter.setRightFlywheelPower(-ShooterSubsystem.FLYWHEEL_MAX_SHOOT_POWER);
 			}
 			else{
 				Robot.shooter.setRightFlywheelPower(-ShooterSubsystem.FLYWHEEL_MIN_SHOOT_POWER);
 			}
    	}
 		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return running;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.setLeftFlywheelPower(0);
    	Robot.shooter.setRightFlywheelPower(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
