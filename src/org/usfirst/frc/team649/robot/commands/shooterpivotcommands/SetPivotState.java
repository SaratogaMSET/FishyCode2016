package org.usfirst.frc.team649.robot.commands.shooterpivotcommands;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
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
			
		} else if (state == ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_POSITION;
			
		} else if (state == ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.FAR_SHOOT_POSITION;
			
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
		pid.enable();
		
		if (!Robot.intake.isIntakeDeployed()){
			if (Robot.shooterPivot.getPosition() > PivotPID.TOP_OF_INTAKE_ZONE && setPoint < PivotPID.TOP_OF_INTAKE_ZONE){
				inDangerOfIntakes = true;
			}
			
			if (Robot.shooterPivot.getPosition() < PivotPID.BOTTOM_OF_INTAKE_ZONE && setPoint > PivotPID.BOTTOM_OF_INTAKE_ZONE){
				inDangerOfIntakes = true;
			}
		}
		
		if (up && setPoint == PivotPID.STORE_POSITION){
			PivotPID.max_motor_up_power = PivotPID.MIDDLE_STATE_MAX_UP_POWER;
		}
		else {
			PivotPID.max_motor_up_power = PivotPID.REGULAR_MAX_UP_POWER;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putString("Current Command", "Set Pivot");
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
		
		System.out.println("Entered Pivot execute");
		
		averageMotorSpeed = (Math.abs(Robot.shooterPivot.motorLeft.get()) + Math.abs(Robot.shooterPivot.motorRight.get())) / 2.0;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// inDangerOfIntakes = Robot.intake.isIntakeDeployed(); //TODO any
		// ending cases not below
		// end if (TODO end cases), if going up and past max, if going down and at bumpers, 
		//if pid is done, or if it's been going for too long, if applying insignificant power
		//END
		return inDangerOfIntakes || up && Robot.shooterPivot.pastMax() || !up
				&& Robot.shooterPivot.lowerLimitsTriggered() || pid.onTarget() ||
				timer.get() > 2.5 || isStalling || (timer.get() > 0.8 && Math.abs(averageMotorSpeed) < PivotPID.MIN_PIVOT_SPEED)
				|| Robot.oi.driver.isManualOverride();
	}
	
	// Called once after isFinished returns true
	protected void end() {
		pid.disable();
		Robot.shooterPIDIsRunning = false;
		SmartDashboard.putString("  Current Command", " ");
		// Robot.shooterPivot.setPower(0);
		// Robot.shooterPivot.engageBrake(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
