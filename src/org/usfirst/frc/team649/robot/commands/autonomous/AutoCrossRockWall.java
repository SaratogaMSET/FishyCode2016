package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveWithAccelerometerRockWall;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossRockWall extends CommandGroup {
	public AutoCrossRockWall(){
		addSequential(new ResetDTEncoders());
		addSequential(new ShiftDrivetrain(false));
		addSequential(new ResetPivot());
		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE));
		addSequential(new DriveWithAccelerometerRockWall());
		//addSequential(new DrivetrainPIDCommand(40));
	}
}
