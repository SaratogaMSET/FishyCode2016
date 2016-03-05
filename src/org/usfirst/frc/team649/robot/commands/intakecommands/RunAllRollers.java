package org.usfirst.frc.team649.robot.commands.intakecommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RunAllRollers extends Command {
	int dir;
	boolean untilIR;
	boolean isCenterersStalling, prevStateTooMuchCurrent;
	Timer intakeTimer;

	//representing the preset direction, IN = 0, OUT = 1, OFF = 2
    public RunAllRollers(int dir, boolean untilIR) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.dir = dir;
    	this.untilIR = untilIR;
    	isCenterersStalling = false;
    	prevStateTooMuchCurrent = false;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	intakeTimer = new Timer();
    	
    	if (dir == ShooterSubsystem.IN){
	    	Robot.shooter.setLeftFlywheelPower(ShooterSubsystem.FLYWHEEL_INTAKE_POWER);
	    	Robot.shooter.setRightFlywheelPower(-ShooterSubsystem.FLYWHEEL_INTAKE_POWER);
	    	
	    	Robot.intake.setFwdRolSpd(IntakeSubsystem.FORWARD_ROLLER_INTAKE_SPEED);
	    	Robot.intake.setCenteringModuleSpeed(IntakeSubsystem.CENTERING_MODULE_INTAKE_SPEED);
    	}
    	else if (dir == ShooterSubsystem.OUT){
    		//inverse of above basically
    		Robot.shooter.setLeftFlywheelPower(-ShooterSubsystem.FLYWHEEL_INTAKE_POWER);
	    	Robot.shooter.setRightFlywheelPower(ShooterSubsystem.FLYWHEEL_INTAKE_POWER);
	    	
	    	Robot.intake.setFwdRolSpd(IntakeSubsystem.FORWARD_ROLLER_PURGE_SPEED);
	    	Robot.intake.setCenteringModuleSpeed(IntakeSubsystem.CENTERING_MODULE_PURGE_SPEED);
    	}
    	else{
    		Robot.shooter.setLeftFlywheelPower(0);
	    	Robot.shooter.setRightFlywheelPower(0);
	    	
	    	Robot.intake.setFwdRolSpd(0);
	    	Robot.intake.setCenteringModuleSpeed(0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if (Robot.pdp.getCurrent(RobotMap.Intake.PDP_PORTS[0]) > IntakeSubsystem.CURRENT_THRESHOLD
    			|| Robot.pdp.getCurrent(RobotMap.Intake.PDP_PORTS[1]) > IntakeSubsystem.CURRENT_THRESHOLD){
			if (intakeTimer.get() > IntakeSubsystem.MIN_STALL_TIME){
				isCenterersStalling = true;
			}
			//if current stalling for the first time, start the timer
			if (!prevStateTooMuchCurrent){
				intakeTimer.reset();
				intakeTimer.start();
			}
			//keep this at the end
			prevStateTooMuchCurrent = true;
		}
		else{
			isCenterersStalling = false;
			//keep this at the end
			intakeTimer.reset();
			prevStateTooMuchCurrent = false;
		}
    	

    	SmartDashboard.putString("INTAKE Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (untilIR){
        	//end if IR has been tripped (false)
        	return !Robot.shooter.infraredSensor.get() || isCenterersStalling
        			|| Robot.oi.driver.isManualOverride();
        }
        else{
        	return true;
        }
    }

    // Called once after isFinished returns true
    protected void end() {
    	if (untilIR){
    		Robot.shooter.setLeftFlywheelPower(0);
	    	Robot.shooter.setRightFlywheelPower(0);
	    	
	    	Robot.intake.setFwdRolSpd(0);
	    	Robot.intake.setCenteringModuleSpeed(0);
    	}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
