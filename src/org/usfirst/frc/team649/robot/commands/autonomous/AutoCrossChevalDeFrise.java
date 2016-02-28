package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DriveTrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.Wait;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		addSequential(new DriveTrainPIDCommand(84));
		addSequential(new SetIntakePosition(false));
		addSequential(new DriveTrainPIDCommand(20));
		addSequential(new Wait(500.0));
		addSequential(new DriveTrainPIDCommand(78));
		//addParallel(new putColordown);
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
	
}
