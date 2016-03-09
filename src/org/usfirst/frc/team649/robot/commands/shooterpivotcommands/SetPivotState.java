package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetPivotState extends Command {

	PIDController pid;
	boolean inDangerOfIntakes, up;
	boolean exit;
	double setPoint;
	int state;
	double averageMotorSpeed;
	Timer timer;
	boolean isStalling; // set to true if current is greater than constant
	//int oldState;

	// in ShooterPivot SubSystem and if not moving by some amount (constant in
	// shooter subsystem)
	// per second. End command after.

	public SetPivotState(int state) {
		requires(Robot.shooterPivot);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		pid = Robot.shooterPivot.getPIDController();

		this.state = state;
		
		System.out.println("Entered Pivot state");
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		//oldState = Robot.shooterPivot.currentPivotState;
		Robot.shooterPivot.currentPivotState = state;
		
		if (state == ShooterPivotSubsystem.PivotPID.PICKUP_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.PICKUP_POSITION;
			
		} else if (state == ShooterPivotSubsystem.PivotPID.STORING_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.STORE_POSITION;
			
		} else if (state == ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.FAR_SHOOT_POSITION;
			
		} else if (state == ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_POSITION;
			
		} else if (state == ShooterPivotSubsystem.PivotPID.BACK_SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.BACK_SHOOT_POSITION;
			
		} else {
			setPoint = Robot.shooterPivot.getPosition();
		}

		up = setPoint > Robot.shooterPivot.getPosition();
		
		timer = new Timer();
		timer.start();
		inDangerOfIntakes = false;
		Robot.shooterPIDIsRunning = true;
		pid.setSetpoint(setPoint);
		
		if (!Robot.intake.isIntakeDeployed()){
			if ((Robot.shooterPivot.getPosition() > PivotPID.TOP_OF_INTAKE_ZONE && setPoint < PivotPID.TOP_OF_INTAKE_ZONE)
					|| (Robot.shooterPivot.getPosition() < PivotPID.BOTTOM_OF_INTAKE_ZONE && setPoint > PivotPID.BOTTOM_OF_INTAKE_ZONE)){
				//inDangerOfIntakes = true;
				//new SetIntakePosition(!IntakeSubsystem.UP).start();
				System.out.println("Entered the deploy intake section");
				Robot.intake.setSolenoids(true);
				Robot.intakeState = true;
			
				Timer t = new Timer();
				t.start();
				boolean exit = false;
				while (!exit && !Robot.intake.isIntakeDeployed()){ //wait for intakes to be down, t
					System.out.println("waiting for intakes... Time: " + t.get());
					if (t.get() > 2.0){
						exit = true;
						System.out.println("Timed out at t = " + t.get());
					}
				}
			}
		}
	
		pid.enable();

		if (up && setPoint == PivotPID.STORE_POSITION){
			PivotPID.max_motor_up_power = PivotPID.MIDDLE_STATE_MAX_UP_POWER;
		}
		else {
			
			PivotPID.max_motor_up_power = PivotPID.REGULAR_MAX_UP_POWER;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.shooterPIDIsRunning = true;
		
		if (!Robot.intake.isIntakeDeployed()) {

			if ((Robot.shooterPivot.isInIntakeZone() && Robot.shooterPivot.getPosition() > ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE
					&& setPoint < ShooterPivotSubsystem.PivotPID.TOP_OF_INTAKE_ZONE && !up)
					|| (Robot.shooterPivot.isInIntakeZone() && Robot.shooterPivot.getPosition() < ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE
							&& setPoint > ShooterPivotSubsystem.PivotPID.BOTTOM_OF_INTAKE_ZONE && up)) {
				//what issue are you seeing, not doing down to top of danger zone. or up when above
				inDangerOfIntakes = true;
			}
			
	
		}
		
		if (Robot.shooterPivot.averageMotorCurrents() >= ShooterPivotSubsystem.PivotPID.CURRENT_LIMIT
				&& Robot.shooterPivot.getEncoderRate() <= ShooterPivotSubsystem.PivotPID.MINIMUM_ENCODER_RATE){
			//isStalling = true;
		}
		
		SmartDashboard.putString("SHOOTERPIVOT Current Command", this.getName());
		
		averageMotorSpeed = (Math.abs(Robot.shooterPivot.motorLeft.get()) + Math.abs(Robot.shooterPivot.motorRight.get())) / 2.0;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// inDangerOfIntakes = Robot.intake.isIntakeDeployed(); //TODO any
		// ending cases not below
		// end if (TODO end cases), if going up and past max, if going down and at bumpers, 
		//if pid is done, or if it's been going for too long, if applying insignificant power
		//END
		exit = inDangerOfIntakes || up && Robot.shooterPivot.pastMax() || !up
				&& Robot.shooterPivot.lowerLimitsTriggered() || Robot.shooterPivot.isOnTarget(setPoint) ||
				timer.get() > 5.0 || isStalling || (timer.get() > 0.8 && Math.abs(averageMotorSpeed) < PivotPID.MIN_PIVOT_SPEED)
				|| Robot.oi.driver.isManualOverride();
		return exit;
	}
	
	// Called once after isFinished returns true
	protected void end() {
		//do this to minimize time pivot has power of zero and prevent slip
		pid.disable();
		double powerUpOrDown = 1.0;
		if(Robot.shooterPivot.getPivotAngle() > 80) {
			powerUpOrDown = -1.0;
		}
		Robot.shooterPivot.setPower(powerUpOrDown*ShooterPivotSubsystem.PivotPID.HOLD_PIVOT_POSITION_POWER);
		Robot.shooterPIDIsRunning = false;
		SmartDashboard.putString("  Current Command", " "+ "");
		System.out.println("end: " + timer.get() );
		System.out.println("Pivot angle " + Robot.shooterPivot.returnPIDInput());
		// Robot.shooterPivot.setPower(0);
		// Robot.shooterPivot.engageBrake(true);
		System.out.println("inDangerOfIntakes: " + inDangerOfIntakes + ", EXIT: " + exit);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
