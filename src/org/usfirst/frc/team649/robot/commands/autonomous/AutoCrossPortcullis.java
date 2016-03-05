package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoCrossPortcullis extends CommandGroup {
	
	public AutoCrossPortcullis() {
		
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_START_TO_RAMP_PORTCULLIS));
		addSequential(new WaitCommand(2));
		addSequential(new SetIntakePosition(true));
		addSequential(new WaitCommand(0.5)); //make sure don't ram into Portcullis
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_END_PORTCULLIS)); //faster speed needed?
		addSequential(new SetIntakePosition(false));
		//color sensor?
			
	}

}
