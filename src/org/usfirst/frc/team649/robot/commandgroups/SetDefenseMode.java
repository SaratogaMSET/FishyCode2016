package org.usfirst.frc.team649.robot.commandgroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class SetDefenseMode extends CommandGroup {
	public SetDefenseMode() {
		// TODO Auto-generated constructor stub
		
		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE));
		addSequential(new SetIntakePosition(IntakeSubsystem.UP));
		
	}
	
	@Override
	public boolean isFinished(){
		return Robot.oi.driver.isManualOverride();
		
	}

}
