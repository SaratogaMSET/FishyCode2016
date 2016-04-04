package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.TurnWithVision;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPosition;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.subsystems.CameraSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoShootSequence extends CommandGroup {
	public AutoShootSequence(int pos){
		double turnAngle; //must be under the actual turn distance
		double pivotAngle;
		switch (pos){
			case 1:
				turnAngle = AutoConstants.TURN_FROM_POS_1;
				pivotAngle = PivotPID.AUTO_POS1_SHOOT_POSITION;
				break;
			case 2:
				turnAngle = AutoConstants.TURN_FROM_POS_2;
				pivotAngle = PivotPID.AUTO_POS2_SHOOT_POSITION;
				break;
			case 3:
				turnAngle = AutoConstants.TURN_FROM_POS_3;
				pivotAngle = PivotPID.AUTO_POS3_SHOOT_POSITION;
				break;
			case 4:
				turnAngle = AutoConstants.TURN_FROM_POS_4;
				pivotAngle = PivotPID.AUTO_POS4_SHOOT_POSITION;
				break;
			default:
				turnAngle = AutoConstants.TURN_FROM_POS_1;
				pivotAngle = PivotPID.AUTO_POS1_SHOOT_POSITION;
				break;
		}
		
		addSequential(new SetCameraPiston(CameraSubsystem.CAM_UP));
		
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new TurnWithEncoders(turnAngle)); //time out
		addSequential(new DriveForwardRotate(0, 0));

		addSequential(new WaitCommand(0.5));
		//different than SetPivotState, this allows for a non state machine
		addSequential(new SetPivotPosition(pivotAngle));
		
		addSequential(new TurnWithVision()); //has a built in wait
		addSequential(new TurnWithVision()); //in case of overshoot

		
		addSequential(new WaitCommand(1.0));
		addSequential(new BangBangFlywheels(false)); //false for not in teleop
		addSequential(new WaitCommand(1.5));
		addSequential(new ResetPivot());
	}
}
