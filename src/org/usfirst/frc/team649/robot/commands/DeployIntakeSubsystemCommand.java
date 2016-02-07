package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DeployIntakeSubsystemCommand extends Command {

	IntakeSubsystem intake = new IntakeSubsystem();

	public DeployIntakeSubsystemCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		super("Deploy Intake using Double Solenoids and Rollers");
		requires(intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		intake.setFwdRolSpd(0);
		intake.setCenteringModuleSpeed(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		intake.setFwdRolSpd(0);
		intake.setCenteringModuleSpeed(0);
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
