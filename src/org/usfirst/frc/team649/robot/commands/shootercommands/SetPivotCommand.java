package org.usfirst.frc.team649.robot.commands.shootercommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetPivotCommand extends Command {

	PIDController pid;
	
	boolean end, onlyUp, onlyDown;
	double setPoint;
	Timer timer;
    
	public SetPivotCommand(int state) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	pid = Robot.shooterPivot.getPIDController();
    	
    	if(state==ShooterPivotSubsystem.PivotPID.PICKUP_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.PICKUP_POSITION;
    	}else if(state==ShooterPivotSubsystem.PivotPID.STORING_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.STORE_POSITION;
    	}else if(state==ShooterPivotSubsystem.PivotPID.SHOOT_STATE){
    		setPoint = ShooterPivotSubsystem.PivotPID.SHOOT_POSITION;
    	}else{
    		setPoint = Robot.shooterPivot.getPosition();
    	}
    	
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer = new Timer();
    	timer.start();
    	end = false;
    	Robot.shooterPivot.engageBrake(false);
    	pid.setSetpoint(setPoint);
    	pid.enable();
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putString("Current Command", "Set Pivot");
    	
    	if(!Robot.intake.isIntakeDeployed() && Robot.shooterPivot.){
    		end = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (end || pid.onTarget() || timer.get() >3.0);
    }

    // Called once after isFinished returns true
    protected void end() {
    	pid.disable();
		SmartDashboard.putString("Current Command", " ");
    	Robot.shooterPivot.setPower(0);
    	Robot.shooterPivot.engageBrake(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooterPivot.motorLeft.set(0);
    	Robot.shooterPivot.motorRight.set(0);
    }
}
