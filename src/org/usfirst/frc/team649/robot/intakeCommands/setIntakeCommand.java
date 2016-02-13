package org.usfirst.frc.team649.robot.intakeCommands;

import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class setIntakeCommand extends Command {

	DrivetrainSubsystem drive;
    public setIntakeCommand() {
    	drive = new DrivetrainSubsystem();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drive.driveSol.set(!drive.driveSol.get());
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
