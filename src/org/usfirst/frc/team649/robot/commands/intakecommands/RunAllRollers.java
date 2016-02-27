package org.usfirst.frc.team649.robot.commands.intakecommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunAllRollers extends Command {
	int dir;
	boolean untilIR;

	//representing the preset direction, IN = 0, OUT = 1, OFF = 2
    public RunAllRollers(int dir, boolean untilIR) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.dir = dir;
    	this.untilIR = untilIR;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (untilIR){
        	//end if IR has been tripped (false)
        	return !Robot.shooter.infraredSensor.get();
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
