package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;

import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.SetPivotPower;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakeSpeed;
import org.usfirst.frc.team649.robot.commands.shootercommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shootercommands.SetPivotState;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static DrivetrainSubsystem drivetrain;
	public static IntakeSubsystem intake;
	public static ShooterPivotSubsystem shooterPivot;
	public static ShooterSubsystem shooter;
	public ArrayList<ArrayList<Double>> log;
	public static Timer timer;
	public DoubleSolenoid ds;
	public PowerDistributionPanel pdp;

	SendableChooser chooser;
	
	public static double DEAD_ZONE_TOLERANCE = 0.025;
	
	//true = down, false = up
	public static boolean intakeState = false;
	//true = high gear, false = low gear
	public static boolean currentGear = true;
	public static boolean shooterPIDIsRunning = false;
	
	//prevStates
	public boolean prevStateOpTrigger;
	public boolean prevStateDriveShifter;
	public boolean tempPrevState;

	
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivetrain = new DrivetrainSubsystem();
		intake = new IntakeSubsystem();
		shooterPivot = new ShooterPivotSubsystem();
		shooter = new ShooterSubsystem();
		log = new ArrayList<>();
		timer = new Timer();
		pdp = new PowerDistributionPanel();
		
		prevStateOpTrigger = false;
		prevStateDriveShifter = false;
		tempPrevState = false;
	}

	public void disabledInit() {
		System.out.println("STARTING LOG: Time, MotorLeft, MotorRight ");
		for (int i = 0; i < log.size(); i++) {
			ArrayList<Double> d = log.get(i);
			System.out.println("BEGINNING_TAG " + d.get(0) + ", " + d.get(1)
					+ ", " + d.get(2) + ", " + d.get(3) + ", " + d.get(4)
					+ ", " + d.get(5) + " ENDING_TAG");
		}
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	
	public void autonomousInit() {

	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		timer.reset();
		timer.start();
		log = new ArrayList<>();
		drivetrain.gyro.reset();
		shooterPivot.resetEncoders();
		//new SetPivotCommand(5).start();;
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		new DriveForwardRotate(oi.driver.getForward(), oi.driver.getRotation()).start();
		
		//INTAKE ----- toggle
		if (oi.operator.toggleIntake()) { //&& !prevStateOpTrigger) {
			shooter.loadBall(DoubleSolenoid.Value.kForward);
			///	new SetPivotCommand(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
			//new SetIntakePositionCommand(intakeState).start();
			intakeState = !intakeState;
		} else {
			shooter.loadBall(DoubleSolenoid.Value.kReverse);

		}
		
		
		
		
		if(oi.operatorJoystick.getRawButton(4)) {//&& !tempPrevState){
			shooter.setLeftFlywheelPower(0.50);
			shooter.setRightFlywheelPower(-0.50);
			//	new SetPivotCommand(ShooterPivotSubsystem.PivotPID.SHOOT_STATE).start();
		} else if(oi.operatorJoystick.getRawButton(6)) {
			shooter.setLeftFlywheelPower(-1.0);
			shooter.setRightFlywheelPower(1.0);
		} else {
			shooter.setLeftFlywheelPower(0.0);
			shooter.setRightFlywheelPower(0.0);
		}
		
		if(oi.operator.purgeIntake()) {
			new SetIntakeSpeed(IntakeSubsystem.FORWARD_ROLLER_PURGE_SPEED,
					IntakeSubsystem.CENTERING_MODULE_PURGE_SPEED).start(); 
		} else if (oi.operator.runIntake()) {
			new SetIntakeSpeed(IntakeSubsystem.FORWARD_ROLLER_INTAKE_SPEED,
					IntakeSubsystem.CENTERING_MODULE_INTAKE_SPEED).start();
		} else {
			new SetIntakeSpeed(IntakeSubsystem.INTAKE_OFF_SPEED,
					IntakeSubsystem.INTAKE_OFF_SPEED).start();
		}
		
		if(oi.driver.isDrivetrainLowGearButtonPressed() && !prevStateDriveShifter){
			drivetrain.shift(!currentGear);
			currentGear = !currentGear;
		} else {
			drivetrain.shift(currentGear);
		}
		
		if(oi.operatorJoystick.getRawButton(2) && !shooterPIDIsRunning) {
			double pivotPower = correctForDeadZone(oi.operatorJoystick.getY()/2.0);
			
			if (pivotPower > 0 && shooterPivot.pastMax() || pivotPower < 0 && shooterPivot.bumpersTriggered()){
				pivotPower = 0;
			}
			
			new SetPivotPower(pivotPower).start();
		}
		//LOGGING AND DASHBOARD
		log.add(drivetrain.getLoggingData());
		
		SmartDashboard.putData("Shooter Pivot left", shooterPivot.encoderLeft);
		SmartDashboard.putData("Shooter Pivot Right", shooterPivot.encoderRight);
		
		SmartDashboard.putBoolean("Shooter bumper left", shooterPivot.resetBumperLeft.get());
		SmartDashboard.putBoolean("Shooter bumper right", shooterPivot.resetBumperRight.get());
		
		SmartDashboard.putNumber("Shooter Pivot current right", pdp.getCurrent(0));
		SmartDashboard.putNumber("Shooter pivot current left", pdp.getCurrent(15));
		/*
		SmartDashboard.putData("GYRO", drivetrain.gyro);
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());

		SmartDashboard.putData("Left Enc", drivetrain.leftEncoder);
		SmartDashboard.putData("Right Enc", drivetrain.rightEncoder);
		SmartDashboard.putBoolean("current gear", currentGear);
		SmartDashboard.putBoolean("current intake state", intakeState);
*/
		SmartDashboard.putString("Current Command", " ");
		
		SmartDashboard.putBoolean("Shooter Hall", shooterPivot.reachedResetLimit());//shooterPivot.resetHalEffect.getDirection());
		//SmartDashboard.putNumber("Shooter Hall Effect", shooterPivot.resetHalEffect.get());//shooterPivot.resetHalEffect.getDirection());

		//PREV STATES
		prevStateOpTrigger = oi.operatorJoystick.getRawButton(1);
		prevStateDriveShifter = oi.driver.isDrivetrainLowGearButtonPressed();
		tempPrevState = oi.operatorJoystick.getRawButton(4);
		
		//********updating subsystem*******//
		
		//shooter hal effect counter
		shooterPivot.updateHalEffect();
		//brake
		if (shooterPivot.motorLeft.get() == 0 && shooterPivot.motorRight.get() == 0){
			shooterPivot.engageBrake(true);
		}
		else{
			shooterPivot.engageBrake(false);
		}
	}

	
	public double correctForDeadZone(double joyVal){
		   return Math.abs(joyVal) >= DEAD_ZONE_TOLERANCE ? joyVal : 0;
   }
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
