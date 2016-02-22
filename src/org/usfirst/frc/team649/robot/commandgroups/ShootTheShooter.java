package org.usfirst.frc.team649.robot.commandgroups;


import org.usfirst.frc.team649.robot.shootercommands.ShooterSet;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ShootTheShooter extends CommandGroup {
	public ShootTheShooter(){
		addSequential(new ShooterSet(DoubleSolenoid.Value.kForward));
		addSequential(new WaitCommand(1.0));
		addSequential(new ShooterSet(DoubleSolenoid.Value.kReverse));
	}
}
