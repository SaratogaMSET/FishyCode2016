package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPosition;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoShootSequence extends CommandGroup {
	public AutoShootSequence(int pos){
		double turnAngle;
		switch (pos){
			case 1:
				turnAngle = AutoConstants.TURN_FROM_POS_1;
				break;
			case 2:
				turnAngle = AutoConstants.TURN_FROM_POS_2;
				break;
			case 3:
				turnAngle = AutoConstants.TURN_FROM_POS_3;
				break;
			case 4:
				turnAngle = AutoConstants.TURN_FROM_POS_4;
				break;
			default:
				turnAngle = 0;
				break;
		}
		
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new TurnWithEncoders(turnAngle), 7);
		addSequential(new DriveForwardRotate(0, 0));
		//insert VISION turn here

		//different than SetPivotState, this allows for a non state machine
		addSequential(new SetPivotPosition(PivotPID.AUTO_LOW_BAR_SHOOT_POSITION));
		addSequential(new WaitCommand(1.0));
		addSequential(new BangBangFlywheels(false)); //false for not in teleop
		addSequential(new WaitCommand(1.5));
		addSequential(new ResetPivot());
	}
}
