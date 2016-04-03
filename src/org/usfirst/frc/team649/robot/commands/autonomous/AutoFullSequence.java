package org.usfirst.frc.team649.robot.commands.autonomous;

import org.usfirst.frc.team649.robot.commands.MatchAutoDrive;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem.AutoConstants;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoFullSequence extends CommandGroup {
	public AutoFullSequence(int defense, int pos) {
		//whatever initial commands there are;
		
		//choose the pos
		switch (defense){
		case AutoConstants.LOW_BAR:
			addSequential(new AutoCrossLowBar());
			break;
		case AutoConstants.CHEVAL:
			addSequential(new AutoCrossChevalDeFrise());
			break;
		case AutoConstants.PORTCULLIS:
			addSequential(new AutoCrossPortcullis());
			break;
		case AutoConstants.ROCK_WALL:
			addSequential(new AutoCrossRockWall());
			break;
		case AutoConstants.ROUGH_TERRAIN:
			addSequential(new AutoCrossRoughTerrain());
			break;
		default:
			return;
	}
		
		
	//shoot from the pos
	addSequential(new AutoShootSequence(pos));
		
		
		
		
		
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
