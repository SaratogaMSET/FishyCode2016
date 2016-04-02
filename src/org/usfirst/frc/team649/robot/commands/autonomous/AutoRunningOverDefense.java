package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoRunningOverDefense extends CommandGroup{
	public AutoRunningOverDefense(){
		addSequential(new ResetPivot());
		//addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE));
		addSequential(new DrivetrainPIDCommand(181));
	}
}
