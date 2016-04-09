package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoFullSequence extends CommandGroup {
	public AutoFullSequence(int defense, int pos) {
		//whatever initial commands there are;
		System.out.println("\n");
		
		if (pos == AutoConstants.DO_NOTHING || defense == AutoConstants.DO_NOTHING){
			System.out.println("DOING NOTHING");
			Robot.logMessage("Running NO autonomous: POS:  " + pos + " , Defense: " + defense);
			return;
		}
		
		if (pos == AutoConstants.POS1 && defense != AutoConstants.LOW_BAR 
				|| pos != AutoConstants.POS1 && defense == AutoConstants.LOW_BAR){
			System.out.println("tried doing either -- pos 1 and not low bar, or -- low bar and not pos 1");
			Robot.logMessage("Running NO autonomous: POS:  " + pos + " , Defense: " + defense);
			return;
		}
		
		//choose the pos
		switch (defense){
		case AutoConstants.LOW_BAR:
			System.out.println("do Low Bar");
			addSequential(new AutoCrossLowBar());
			pos = AutoConstants.POS1;
			break;
		case AutoConstants.CHEVAL:
			System.out.println("do Cheval");
			addSequential(new AutoCrossChevalDeFrise());
			break;
		case AutoConstants.PORTCULLIS:
			System.out.println("do Portcullis");
//			addSequential(new AutoCrossPortcullis());
			break;
		case AutoConstants.ROCK_WALL:
			System.out.println("do Rock Wall");
			addSequential(new AutoCrossRockWall());
			break;
		case AutoConstants.ROUGH_TERRAIN:
			System.out.println("do Rough terrain");
			addSequential(new AutoCrossRoughTerrain());
			break;
		default:
			pos = AutoConstants.DO_NOTHING; // redundancy
			System.out.println("Would do NNOTHIINNGG???? why it should've ended already wtf");
			return;
	}
	
	System.out.println("POS:  " + pos + " , Defense: " + defense);
	Robot.logMessage("Running autonomous: POS:  " + pos + " , Defense: " + defense);
		
	//shoot from the pos
	if (pos != AutoConstants.DO_NOTHING){
		addSequential(new AutoShootSequence(pos));
		System.out.println("doing position " + pos + " with no problems");
	}
	
		
	System.out.println("\n");
		
		
		//crossing defenses
//		Command crossingCommand;
//		
//		try{
//			crossingCommand = selectDefenseCommand(defense);
//		}
//		catch (Exception e){
//			System.out.println(e.getMessage());
//			return;
//		}
//		addSequential(crossingCommand);
//		
//		//adjust angle
//		//TODO: Turn Gyro Command
//		
//		//calculate vision offset and start profile
//		switch (pos){
//			case 1:
//				addSequential(new MatchAutoDrive(AutonomousSequences.fromPos1, 1));
//				break;
//			case 2:
//				addSequential(new MatchAutoDrive(AutonomousSequences.fromPos2, 2));
//				break;
//			case 3:
//				addSequential(new MatchAutoDrive(AutonomousSequences.fromPos3, 3));
//				break;
//			case 4:
//				addSequential(new MatchAutoDrive(AutonomousSequences.fromPos4, 4));
//				break;
//			case 5:
//				addSequential(new MatchAutoDrive(AutonomousSequences.fromPos5, 5));
//				break;
//				
//		}
//		addSequential(new SetPivotState(ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_STATE));
//		//addSequential(new RampUpAndShoot()); //TODO create bang bang
//	}
//	
//	public Command selectDefenseCommand(int defense) throws Exception{
//		Command crossingCommand = new AutoCrossChevalDeFrise();
////		switch(defense){
////		case AutoConstants.LOW_BAR:
////			crossingCommand = new AutoCrossLowBar();
////			break;
////		case AutoConstants.PORTCULLIS:
////			crossingCommand = new AutoCrossPortcullis();
////			break;
////		case AutoConstants.CHEVAL_DE_FRISE:
////			crossingCommand = new AutoCrossChevalDeFrise();
////			break;
////		case AutoConstants.ROCK_WALL:
////			crossingCommand = new AutoCrossRockWall();
////			break;
////		case AutoConstants.ROUGH_TERRAIN:
////			crossingCommand = new AutoCrossRoughTerrain();
////			break;
////		case AutoConstants.SALLY_PORT:
////			crossingCommand = new AutoCrossSallyPort();
////			break;
////		case AutoConstants.DRAWBRIDGE:
////			crossingCommand = new AutoCrossDrawbridge();
////			break;
////		case AutoConstants.RAMPARTS:
////			crossingCommand = new AutoCrossRamparts();
////			break;
////		case AutoConstants.MOAT:
////			crossingCommand = new AutoCrossMoat();
////			break;
////		default:
////			throw new Exception("ERROR: PASSED INVALID DEFENSE");
////			
////		}
//		return crossingCommand;
	}
}
