package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.Wait;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.*;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		addParallel(new SetIntakePosition(false));
		addParallel(new ShiftDrivetrain(false));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_START_TO_RAMP_CHEVAL));
		addSequential(new SetIntakePosition(true));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_MIDRAMP_CHEVAL));
		addSequential(new WaitCommand(AutoConstants.WAIT_TIME_AT_TOP_CHEVAL));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_OFF_CHEVAL));
		//addParallel(new putColordown);
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
	
}
