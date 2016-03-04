package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveWithAccelerometerRockWall;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossRockWall extends CommandGroup {
	public AutoCrossRockWall(){
		addSequential(new DriveWithAccelerometerRockWall());
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_OFF_ROCKWALL));
	}
}
