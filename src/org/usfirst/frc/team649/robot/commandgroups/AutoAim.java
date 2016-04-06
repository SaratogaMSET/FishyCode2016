package org.usfirst.frc.team649.robot.commandgroups;

import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.TurnWithVision;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoAim extends CommandGroup {
    
    public  AutoAim() {
    	addParallel(new SetPivotPosition(PivotPID.AUTO_CAMERA_AIM_POSITION));
    //	addSequential(new WaitCommand(.3));
    	//addSequential(new SetPivotState(PivotPID.STORING_STATE));
    	//addSequential(new SetCameraPiston(true));
    	addSequential(new TurnWithVision(.5));
    }
}
