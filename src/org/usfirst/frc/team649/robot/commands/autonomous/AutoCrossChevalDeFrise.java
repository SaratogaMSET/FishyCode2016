package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		addSequential(new ResetDTEncoders());
		addSequential(new ResetPivot());
		addSequential(new ShiftDrivetrain(false));
		addSequential(new SetIntakePosition(true));
		//addSequential(new DrivetrainPIDCommand(215)); 
		
		addSequential(new SetIntakePosition(true));
		
	//	addSequential(new WaitCommand(1.0));
		addSequential(new TurnWithEncoders(60));
		
		/*
		addSequential(new ResetDTEncoders());
		addSequential(new WaitCommand(1.0));
		addSequential(new DrivetrainPIDCommand(AutoConstants.DISTANCE_RAMP_TO_MIDRAMP_CHEVAL));
		addSequential(new WaitCommand(AutoConstants.WAIT_TIME_AT_TOP_CHEVAL));
		addSequential(new DrivetrainPIDCommand(57));
		*/
		//addSequential(new ResetGyro());
//		addSequential(new TurnWithGyro(51));
//		//addSequential(new TurnWithEncoders(37));
//		addSequential(new DriveForwardRotate(0, 0));
//		addSequential(new SetPivotPosition(PivotPID.AUTO_LOW_BAR_SHOOT_POSITION));
//		addSequential(new WaitCommand(1.0));
//		addSequential(new BangBangFlywheels(false));
//		addSequential(new WaitCommand(1.5));
//		addSequential(new ResetPivot());
		//addSequential(new ShootTheShooter());
		
		//addParallel(new putColordown)
		//addParallel(new stopAtColor);
		
		//addSequential(pid6InchesFrom);
	}
	
}
