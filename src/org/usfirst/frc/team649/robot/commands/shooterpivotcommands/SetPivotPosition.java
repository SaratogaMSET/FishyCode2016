package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetPivotPosition extends Command {

	PIDController pid;
	boolean inDangerOfIntakes, up;
	boolean exit;
	double setpoint;
	double averageMotorSpeed;
	Timer timer;
	boolean isStalling; // set to true if current is greater than constant
	//int oldState;

	// in ShooterPivot SubSystem and if not moving by some amount (constant in
	// shooter subsystem)
	// per second. End command after.

	public SetPivotPosition(double setpoint) {
		requires(Robot.shooterPivot);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		pid = Robot.shooterPivot.getPIDController();
		this.setpoint = setpoint;

		System.out.println("Entered Pivot POSITION");
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//oldState = Robot.shooterPivot.currentPivotState;
		

		up = setpoint > Robot.shooterPivot.getPosition();
		
		timer = new Timer();
		timer.start();
		inDangerOfIntakes = false;
		Robot.shooterPIDIsRunning = true;
		pid.setSetpoint(setpoint);
		
		if (!Robot.intake.isIntakeDeployed()){
			if ((Robot.shooterPivot.getPosition() > PivotPID.TOP_OF_INTAKE_ZONE && setpoint < PivotPID.TOP_OF_INTAKE_ZONE)
					|| (Robot.shooterPivot.getPosition() < PivotPID.BOTTOM_OF_INTAKE_ZONE && setpoint > PivotPID.BOTTOM_OF_INTAKE_ZONE)){
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
						inDangerOfIntakes = true;
						System.out.println("Timed out at t = " + t.get());
					}
				}
			}
		}
	
		pid.enable();

		if (up && setpoint == PivotPID.STORE_POSITION){
			PivotPID.max_motor_up_power = PivotPID.MIDDLE_STATE_MAX_UP_POWER;
		}
		else {
			
			PivotPID.max_motor_up_power = PivotPID.REGULAR_MAX_UP_POWER;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.shooterPIDIsRunning = true;
		
		if (!Robot.intake.isIntakeDeployed()) {

			if ((Robot.shooterPivot.isInIntakeZone() && Robot.shooterPivot.getPosition() > ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE
					&& setpoint < ShooterPivotSubsystem.PivotPID.TOP_OF_INTAKE_ZONE && !up)
					|| (Robot.shooterPivot.isInIntakeZone() && Robot.shooterPivot.getPosition() < ShooterPivotSubsystem.PivotPID.MIDDLE_OF_INTAKE_ZONE
							&& setpoint > ShooterPivotSubsystem.PivotPID.BOTTOM_OF_INTAKE_ZONE && up)) {
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
	@Override
	protected boolean isFinished() {
		// inDangerOfIntakes = Robot.intake.isIntakeDeployed(); //TODO any
		// ending cases not below
		// end if (TODO end cases), if going up and past max, if going down and at bumpers, 
		//if pid is done, or if it's been going for too long, if applying insignificant power
		//END
		exit = inDangerOfIntakes || up && Robot.shooterPivot.pastMax() || !up
				&& Robot.shooterPivot.lowerLimitsTriggered() || Robot.shooterPivot.isOnTarget(setpoint) ||
				timer.get() > 4.0 || isStalling
				|| Robot.oi.driver.isManualOverride();
		return exit;
	}
	
	// Called once after isFinished returns true
	@Override
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
	@Override
	protected void interrupted() {
		//end();
	}
}
