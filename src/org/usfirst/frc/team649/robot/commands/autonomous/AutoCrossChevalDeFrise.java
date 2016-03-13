package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.ResetGyro;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.TurnWithGyro;
import org.usfirst.frc.team649.robot.commands.intakecommands.RunAllRollers;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.shootercommands.SetFlywheels;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.*;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		addSequential(new ResetDTEncoders());
		addSequential(new ResetPivot());
		addSequential(new ShiftDrivetrain(false));
		addSequential(new SetIntakePosition(true));
		addSequential(new DrivetrainPIDCommand(215));
		
		addSequential(new SetIntakePosition(true));
		
		//addSequential(new WaitCommand(15.0));
		/*
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_MIDRAMP_CHEVAL));
		addSequential(new WaitCommand(AutoConstants.WAIT_TIME_AT_TOP_CHEVAL));
		addSequential(new DrivetrainPIDCommand(57));
		*/
		//addSequential(new ResetGyro());
		addSequential(new TurnWithGyro(31));
		//addSequential(new TurnWithEncoders(37));
		addSequential(new DriveForwardRotate(0, 0));
		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE));
		addSequential(new BangBangFlywheels(false));
		addSequential(new WaitCommand(1.5));
		addSequential(new ResetPivot());
		addSequential(new ShootTheShooter());
		
		//addParallel(new putColordown)
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
	
}
