package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPosition;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoCrossLowBar extends CommandGroup {
	
	public AutoCrossLowBar(){
		addSequential(new ResetDTEncoders());
		addSequential(new SetIntakePosition(true));
		addSequential(new ResetPivot());
		addSequential(new ShiftDrivetrain(false));
		addSequential(new DrivetrainPIDCommand(215)); 
		
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new TurnWithEncoders(60)); //time out
		addSequential(new DriveForwardRotate(0, 0));
		//insert VISION turn here
		
		//different than SetPivotState, this allows for a non state machine
		addSequential(new SetPivotPosition(PivotPID.AUTO_LOW_BAR_SHOOT_POSITION));
		addSequential(new WaitCommand(1.0));
		addSequential(new BangBangFlywheels(false)); //false for not in teleop
		addSequential(new WaitCommand(1.5));
		addSequential(new ResetPivot());
		
		//addParallel(new putColordown)
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
}
