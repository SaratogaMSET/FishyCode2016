package org.usfirst.frc.team649.robot.commandgroups;

import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commands.EndSemiAuto;
import org.usfirst.frc.team649.robot.commands.intakecommands.RunAllRollers;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakeSpeed;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

public class SemiAutoLoadBall extends CommandGroup {
	public SemiAutoLoadBall() {
		// TODO Auto-generated constructor stub
		
		//deploy intakes
		addSequential(new SetIntakePosition(true));
		addSequential(new WaitCommand(0.1));
		//addSequential(new ResetPivot());
		addSequential(new RunAllRollers(ShooterSubsystem.IN, ShooterSubsystem.UNTIL_IR));
		
		addSequential(new WaitCommand(0.15));
		
		addSequential(new SetIntakeSpeed(IntakeSubsystem.FORWARD_ROLLER_PURGE_SPEED, IntakeSubsystem.CENTERING_MODULE_PURGE_SPEED));
		addSequential(new WaitCommand(0.7));
		addSequential(new SetIntakeSpeed(0,0));
		
		//addSequential(new WaitForChildren());
		
		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE));
		
		addSequential(new EndSemiAuto(true)); //true = set the semi auto varibale to false
	}
}
