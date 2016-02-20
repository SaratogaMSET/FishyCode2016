package org.usfirst.frc.team649.robot.commands.shootercommands;

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
	double averageMotorSpeed;
	Timer timer;
	boolean isStalling; // set to true if current is greater than constant

	// in ShooterPivot SubSystem and if not moving by some amount (constant in
	// shooter subsystem)
	// per second. End command after.

	public SetPivotState(int state) {
		requires(Robot.shooterPivot);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		pid = Robot.shooterPivot.getPIDController();

		if (state == ShooterPivotSubsystem.PivotPID.PICKUP_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.PICKUP_POSITION;
		} else if (state == ShooterPivotSubsystem.PivotPID.STORING_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.STORE_POSITION;
		} else if (state == ShooterPivotSubsystem.PivotPID.SHOOT_STATE) {
			setPoint = ShooterPivotSubsystem.PivotPID.SHOOT_POSITION;
		} else {
			setPoint = Robot.shooterPivot.getPosition();
		}

		up = setPoint > Robot.shooterPivot.getPosition();
		
		
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timer = new Timer();
		timer.start();
		inDangerOfIntakes = false;
		Robot.shooterPivot.engageBrake(false);
		Robot.shooterPIDIsRunning = true;
		pid.setSetpoint(setPoint);
		pid.enable();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putString("Current Command", "Set Pivot");
		Robot.shooterPIDIsRunning = true;
		if (!Robot.intake.isIntakeDeployed()) {

			if ((!Robot.shooterPivot.isAboveIntakeZone() && setPoint < ShooterPivotSubsystem.PivotPID.TOP_OF_INTAKE_ZONE)
					|| (!Robot.shooterPivot.isBelowIntakeZone() && setPoint > ShooterPivotSubsystem.PivotPID.BOTTOM_OF_INTAKE_ZONE)) {
				//what issue are you seeing, not doing down to top of danger zone.
				inDangerOfIntakes = true;
			}
	
			
			averageMotorSpeed = (Math.abs(Robot.shooterPivot.motorLeft.get()) + Math.abs(Robot.shooterPivot.motorRight.get())) / 2.0;
		}
		
		if (Robot.shooterPivot.averageMotorCurrents() >= ShooterPivotSubsystem.PivotPID.CURRENT_LIMIT
				&& Robot.shooterPivot.getEncoderRate() <= ShooterPivotSubsystem.PivotPID.MINIMUM_ENCODER_RATE){
			isStalling = true;
		}
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
				timer.get() > 2.5 || isStalling || (timer.get() > 0.8 && Math.abs(averageMotorSpeed) < PivotPID.MIN_PIVOT_SPEED);
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
