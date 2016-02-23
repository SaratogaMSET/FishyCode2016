package org.usfirst.frc.team649.robot.commandgroups;

import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ReadyToShoot extends CommandGroup {
    
    public  ReadyToShoot() {
    	addSequential(new SetIntakePosition(true));
    	addParallel(new SetPivotState(PivotPID.SHOOT_STATE));
    	addSequential(new BangBangFlywheels(true));
    }
}
