
package org.usfirst.frc.team649.robot.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BangBangFlywheels extends Command {

	boolean hasStartedShot, endOnOneLoop, prevStateIsShooting, shootingSequenceFinished;
	static double waitTime = 1.6;
	
	Timer extraWaitTime;
    public BangBangFlywheels(boolean isTeleop) {
    	requires(Robot.shooter);
    	endOnOneLoop = isTeleop;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	hasStartedShot = false;
    	prevStateIsShooting = false;
    	shootingSequenceFinished = false;
    	extraWaitTime = new Timer();
    	extraWaitTime.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

		if(Robot.shooter.getLeftFlywheelRPM() <= ShooterSubsystem.flywheelTargetRPM){
			Robot.shooter.setLeftFlywheelPower(ShooterSubsystem.flywheelMaxShootPower);
		}	
		else{
			Robot.shooter.setLeftFlywheelPower(ShooterSubsystem.flywheelMinShootPower);
		}
		
		if(Robot.shooter.getRightFlywheelRPM() <= ShooterSubsystem.flywheelTargetRPM){
			Robot.shooter.setRightFlywheelPower(-ShooterSubsystem.flywheelMaxShootPower);
		}
		else{
			Robot.shooter.setRightFlywheelPower(-ShooterSubsystem.flywheelMinShootPower);
		}
		
		//if in autonomous loop, shoot the shooter when we reach our target RPM
		if (!endOnOneLoop){
			if (!hasStartedShot && (Math.abs(Robot.shooter.getLeftFlywheelRPM() - ShooterSubsystem.flywheelTargetRPM) < ShooterSubsystem.flywheelTolerance
					&& Math.abs(Robot.shooter.getRightFlywheelRPM() - ShooterSubsystem.flywheelTargetRPM) < ShooterSubsystem.flywheelTolerance)){
				new ShootTheShooter().start();
				hasStartedShot = true;
				extraWaitTime.reset();
				extraWaitTime.start();
			}
		}
 		
		SmartDashboard.putString("SHOOTER Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (endOnOneLoop){
        	return true;
        }
        else{
        	if (!Robot.isShooting && prevStateIsShooting){
        		shootingSequenceFinished = true;
        		System.out.println("Finished shooooting");
        	}
        	prevStateIsShooting = Robot.isShooting;
        	System.out.println("Ready: " + shootingSequenceFinished + ", TIME " + Math.round(extraWaitTime.get()) + ", hasShot: " + hasStartedShot);
        	return shootingSequenceFinished || extraWaitTime.get() >  waitTime; //hasShot as a safeguard
        	
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    	if (!endOnOneLoop){
    		//if looping, at the end we can assume we want to set all motors to 0,
    		Robot.shooter.setLeftFlywheelPower(0);
        	Robot.shooter.setRightFlywheelPower(0);
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooter.setLeftFlywheelPower(0);
    	Robot.shooter.setRightFlywheelPower(0);
    }
}
