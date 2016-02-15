package org.usfirst.frc.team649.robot.commands.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class setPivotCommand extends Command {

	int currentState;
	PIDController pidLeft,pidRight;
	boolean end;
	double setPoint;
    public setPivotCommand(int state) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	currentState = state;
    	end = false;
    	if(currentState==ShooterPivotSubsystem.PivotPID.PICKUP_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.PIVOT_PICKUP;
    	}else if(currentState==ShooterPivotSubsystem.PivotPID.STORING_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.PIVOT_STORING;
    	}else if(currentState==ShooterPivotSubsystem.PivotPID.RELEASE_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.PIVOT_RELEASE;
    	}else{
    		end = true;
    	}
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	pidLeft = Robot.shooterPivot.getPIDController();
    	pidLeft.setSetpoint(setPoint);
    	pidRight = Robot.shooterPivot.getPIDController();
    	pidRight.setSetpoint(setPoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(pidLeft.onTarget()){
    		pidLeft.disable();
    		Robot.shooterPivot.motorLeft.set(0);
    	}
    	if(pidRight.onTarget()){
    		pidRight.disable();
    		Robot.shooterPivot.motorRight.set(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (pidLeft.onTarget()&&pidRight.onTarget())||end;
    }

    // Called once after isFinished returns true
    protected void end() {
    	pidLeft.disable();
    	pidRight.disable();
    	Robot.shooterPivot.motorLeft.set(0);
    	Robot.shooterPivot.motorRight.set(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooterPivot.motorLeft.set(0);
    	Robot.shooterPivot.motorRight.set(0);
    }
}
