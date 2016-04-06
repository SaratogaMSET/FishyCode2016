package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.CameraSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossRoughTerrain extends CommandGroup {
	public AutoCrossRoughTerrain(){
		addSequential(new ResetDTEncoders());
		addSequential(new ShiftDrivetrain(false));
		addSequential(new SetCameraPiston(!CameraSubsystem.CAM_UP));
		addSequential(new ResetPivot());
		addSequential(new SetPivotState(PivotPID.STORING_STATE));
		addSequential(new SetIntakePosition(false));
		addSequential(new DrivetrainPIDCommand(145));
	}
}
