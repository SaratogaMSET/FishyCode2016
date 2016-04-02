package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ResetPivot extends Command {
	
	boolean inDangerOfIntakes = false;
	
    public ResetPivot() {
    	requires(Robot.shooterPivot);
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	Robot.shooterPivot.currentPivotState = PivotPID.PICKUP_STATE;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	
    	Robot.shooterPivot.setPower(ShooterPivotSubsystem.PivotPID.ZEROING_CONSTANT_MOVE_POWER);
    	Robot.shooterPIDIsRunning = true;
    	if(!Robot.intakeState //TODO change to isIntakeDeployed when sensors are added
    			&& Robot.shooterPivot.getPosition() > ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE) {
    		inDangerOfIntakes = true;
    	}
    	
//    	if (Robot.semiAutoIsRunning){ //TODO get rid of this stupid thing when we have actual sensors
//    		inDangerOfIntakes = false;
//    	}
    	
    	SmartDashboard.putString("SHOOTERPIVOT Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        return Robot.shooterPivot.lowerLimitsTriggered()|| inDangerOfIntakes
        		|| Robot.oi.driver.isManualOverride() || Robot.shooterPivot.isResetHalTripped();
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    	//Robot.shooterPivot.resetCounter();
    	Robot.shooterPivot.setPower(0);
    	System.out.println(Robot.shooterPivot.getPivotAngle());
    	try {
			Thread.sleep(300);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			
			e.printStackTrace();
			
		}
    	if(!inDangerOfIntakes) {
    		Robot.shooterPivot.resetEncoders();
    	}
    	Robot.shooterPIDIsRunning = false;
    	System.out.println(Robot.shooterPivot.getPivotAngle());

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    	Robot.shooterPivot.setPower(0);
    	Robot.shooterPIDIsRunning = false;
    }
}
