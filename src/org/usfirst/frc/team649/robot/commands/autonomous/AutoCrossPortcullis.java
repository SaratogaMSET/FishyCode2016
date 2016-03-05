package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.Wait;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossPortcullis extends CommandGroup {
	
	public AutoCrossPortcullis() {
		
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_START_TO_RAMP_PORTCULLIS));
		addSequential(new Wait(2));
		addSequential(new SetIntakePosition(true));
		addSequential(new Wait(0.5)); //make sure don't ram into Portcullis
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_END_PORTCULLIS)); //faster speed needed?
		addSequential(new SetIntakePosition(false));
		//color sensor?
			
	}

}
