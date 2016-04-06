package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.DrivetrainPIDCommand;
import org.usfirst.frc.team649.robot.commands.ResetDTEncoders;
import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.ShiftDrivetrain;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.subsystems.CameraSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

public class AutoCrossChevalDeFrise extends CommandGroup {
	
	public AutoCrossChevalDeFrise(){
		//addSequential(pidToRamp7ft);
		addSequential(new ResetDTEncoders());
		addSequential(new SetCameraPiston(!CameraSubsystem.CAM_UP));
		addSequential(new ResetPivot());
		addSequential(new SetIntakePosition(false));
		addSequential(new ShiftDrivetrain(false));
		addSequential(new DrivetrainPIDCommand(180)); 
		
		addSequential(new SetIntakePosition(true));
		
		addSequential(new WaitCommand(1.0));

		addSequential(new ResetDTEncoders());
		addSequential(new DrivetrainPIDCommand(-8.0));
		

		addSequential(new WaitCommand(1.0));
		
		addSequential(new ResetDTEncoders());
		addParallel(new DrivetrainPIDCommand(40)); //AutoConstants.DISTANCE_RAMP_TO_MIDRAMP_CHEVAL
		addSequential(new WaitCommand(0.8)); //waiting time til bring intakes up
		addSequential(new SetIntakePosition(false));
		addSequential(new WaitForChildren()); //wait for driving to stop
		
		addSequential(new WaitCommand(0.75)); //AutoConstants.WAIT_TIME_AT_TOP_CHEVAL
		addSequential(new DrivetrainPIDCommand(100));
		
		//addSequential(new ResetGyro());
//		addSequential(new TurnWithGyro(51));
		//addSequential(new TurnWithEncoders(37));
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
