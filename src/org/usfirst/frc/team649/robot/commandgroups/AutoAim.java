package org.usfirst.frc.team649.robot.commandgroups;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.TurnWithVision;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.TurnConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoAim extends CommandGroup {
    
    public  AutoAim(double velocity) {
//    	addParallel(new SetPivotState(PivotPID.FAR_SHOOT_STATE));
    //	addSequential(new WaitCommand(.3));
    	//addSequential(new SetPivotState(PivotPID.STORING_STATE));
    	//addSequential(new SetCameraPiston(true));
    	addSequential(new TurnWithVision(velocity, !TurnConstants.WAIT_IN_BEGINNING));
    }
    
    @Override
    public void initialize(){
    	Robot.autoAiming = true;
    }
    
    @Override
    protected void interrupted() {
    	end();
    }
    
    @Override
    public void end(){
    	Robot.autoAiming = false;
    }
}
