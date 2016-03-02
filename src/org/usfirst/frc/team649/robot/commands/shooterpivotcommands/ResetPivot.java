package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ResetPivot extends Command {
	
	boolean inDangerOfIntakes = false;
	
    public ResetPivot() {
    	requires(Robot.shooterPivot);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("ACTIVE COMMAND", "Run Til Hall Effect");
    	Robot.shooterPivot.setPower(ShooterPivotSubsystem.PivotPID.ZEROING_CONSTANT_MOVE_POWER);
    	Robot.shooterPIDIsRunning = true;
    	if(!Robot.intakeState //TODO change to isIntakeDeployed when sensors are added
    			&& Robot.shooterPivot.getPosition() > ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE) {
    		inDangerOfIntakes = true;
    	}
    	
//    	if (Robot.semiAutoIsRunning){ //TODO get rid of this stupid thing when we have actual sensors
//    		inDangerOfIntakes = false;
//    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.shooterPivot.lowerLimitsTriggered()|| inDangerOfIntakes
        		|| Robot.oi.driver.isManualOverride();
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.shooterPivot.resetCounter();
    	if(!inDangerOfIntakes) {
    	Robot.shooterPivot.resetEncoders();
    	}
    	Robot.shooterPivot.setPower(0);
    	Robot.shooterPIDIsRunning = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooterPivot.setPower(0);
    	Robot.shooterPIDIsRunning = false;
    }
}
