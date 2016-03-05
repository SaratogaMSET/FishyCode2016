package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithGyro;
import org.usfirst.frc.team649.robot.commands.Wait;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.SetFlywheels;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.*;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		
		addSequential(new ShiftDrivetrain(false));
		addParallel(new SetIntakePosition(true));
		addSequential(new DrivetrainPIDCommand(146));
		
		addSequential(new SetIntakePosition(true));
		/*
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_MIDRAMP_CHEVAL));
		addSequential(new WaitCommand(AutoConstants.WAIT_TIME_AT_TOP_CHEVAL));
		addSequential(new DrivetrainPIDCommand(57));
		*/
		addSequential(new TurnWithGyro(-21.5));
		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE));
		addSequential(new SetFlywheels(1.0, -1.0));
		addSequential(new WaitCommand(2.5));
		addSequential(new ShootTheShooter());
		
		//addParallel(new putColordown)
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
	
}
