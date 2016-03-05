package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DriveWithAccelerometerMoat;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossMoat extends CommandGroup {
	
	public AutoCrossMoat() {
		
		addSequential(new DrivetrainPIDCommand(DrivetrainSubsystem.AutoConstants.DISTANCE_START_TO_RAMP_MOAT));
		addParallel(new DriveForwardRotate(0.6, 0));
		addParallel(new DriveWithAccelerometerMoat());
		addSequential(new DrivetrainPIDCommand(DrivetrainSubsystem.AutoConstants.DISTANCE_RAMP_TO_END_MOAT));
		//color sensor?

	}

}
