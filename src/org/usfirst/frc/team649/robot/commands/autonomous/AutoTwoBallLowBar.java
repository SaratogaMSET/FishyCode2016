package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commandgroups.SemiAutoLoadBall;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithGyro;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.shootercommands.SetFlywheels;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoTwoBallLowBar extends CommandGroup {
    
    public  AutoTwoBallLowBar() {
    	
    	
    	addSequential(new ResetPivot());
		addSequential(new ShiftDrivetrain(false));
		addParallel(new SetIntakePosition(!IntakeSubsystem.UP));
		
		//negeitive bc backwards
		addSequential(new DrivetrainPIDCommand(-DrivetrainSubsystem.AutoConstants.LOW_GOAL_DRIVE_DISTANCE));
		addParallel(new TurnWithGyro(-DrivetrainSubsystem.TurnConstants.LOW_BAR_TURN_ANGLE));
		addParallel(new SetPivotState(PivotPID.BACK_SHOOT_STATE));
		
		addSequential(new BangBangFlywheels(false));
		
		//addSequential(new SetPivotState(PivotPID.STORING_STATE));
		addSequential(new ResetPivot());
		addSequential(new TurnWithGyro(DrivetrainSubsystem.TurnConstants.LOW_BAR_TURN_ANGLE));
		addParallel(new DrivetrainPIDCommand(DrivetrainSubsystem.AutoConstants.TWO_BALL_MIDLINE_DISTANCE));
		
		addSequential(new SemiAutoLoadBall());
		
		
		//FROM HERE ON NEEDS FINISHING
		addParallel(new DrivetrainPIDCommand(-DrivetrainSubsystem.AutoConstants.TWO_BALL_MIDLINE_DISTANCE));
		addSequential(new WaitCommand(2.0));
		addSequential(new SetPivotState(PivotPID.BACK_SHOOT_STATE));
		
		addSequential(new BangBangFlywheels(false));

		
    }
}
