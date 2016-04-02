package org.usfirst.frc.team649.robot.commands.intakecommands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetIntakePosition extends Command {

	boolean deploy;
	
    public SetIntakePosition(boolean toDeploy) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires(Robot.intake);
		deploy = toDeploy;
    }

    // Called just before this Command runs the first time
    @Override
	protected void initialize() {
    	if (!deploy && Robot.shooterPivot.isInIntakeZone()){
    		//don't execute anything if shooter is in the way and we are trying to move up
    	}
    	else{
    		Robot.intake.setSolenoids(deploy); 
    		Robot.intakeState = deploy;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
	protected void execute() {
    	SmartDashboard.putString("INTAKE Current Command", this.getName());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
	protected boolean isFinished() {
        if(deploy) {
        	return Robot.intake.isIntakeDeployed();
        }
        return true;
    }

    // Called once after isFinished returns true
    @Override
	protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
	protected void interrupted() {
    }
}
