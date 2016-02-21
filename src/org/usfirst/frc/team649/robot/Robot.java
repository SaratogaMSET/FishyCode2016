package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.util.Center;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.MatchAutoDrive;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakeSpeed;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPower;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.SetFlywheels;
import org.usfirst.frc.team649.robot.subsystems.CameraSubsystem;
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
	public static CameraSubsystem camera;
	
	public ArrayList<ArrayList<Double>> log;
	public static Timer timer, pivotTimer;
	public DoubleSolenoid ds;
	public static PowerDistributionPanel pdp;

	SendableChooser chooser;
	
	public static double DEAD_ZONE_TOLERANCE = 0.043;
	public static double MIN_STOPPED_TIME = 0.25; //seconds
	
	//true = down, false = up
	public static boolean intakeState = false;
	//true = high gear, false = low gear
	public static boolean currentGear = true;
	public static boolean shooterPIDIsRunning = false;
	public static boolean readyToShoot = false;
	
	//prevStates
	public boolean prevStateOpTrigger;
	public boolean prevStateDriveShifter;
	public boolean tempPrevState;
	public boolean prevStateShootButton;
	
	public boolean prevStateMotorPowerIs0;
	public boolean prevStatePivotUp;
	public boolean prevStateResetButton;
	public boolean prevStatePivotMiddle;

	
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivetrain = new DrivetrainSubsystem();
		intake = new IntakeSubsystem();
		shooterPivot = new ShooterPivotSubsystem();
		shooter = new ShooterSubsystem();
		camera = new CameraSubsystem("http://axis-camera.local");
		
		log = new ArrayList<>();
		timer = new Timer();
		pivotTimer = new Timer();
		pdp = new PowerDistributionPanel();
		
		prevStateOpTrigger = false;
		prevStateDriveShifter = false;
		tempPrevState = false;
		prevStateMotorPowerIs0 = true;
		prevStateShootButton = false;
		prevStatePivotUp = false;
		prevStateResetButton = false;
		prevStatePivotMiddle = false;
		
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
		drivetrain.resetEncoders();
		new DriveForwardRotate(0, 0).start();
		new MatchAutoDrive(AutonomousSequences.fromPos1, 1).start();;

	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		timer.reset();
		timer.start();
		pivotTimer.reset();
		log = new ArrayList<>();
		drivetrain.gyro.reset();
		drivetrain.resetEncoders();
		//shooterPivot.resetEncoders();
		new SetPivotState(5).start();
		new DriveForwardRotate(0, 0).start();;
		
		
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		//new DriveForwardRotate(correctForDeadZone(oi.driver.getForward()), correctForDeadZone(oi.driver.getRotation())).start();
		new DriveForwardRotate(oi.driver.getForward(), oi.driver.getRotation()).start();
		
		//CAMERA
		if (camera.vcap.isOpened()){
			SmartDashboard.putBoolean("Checking For Vision?", true);
			//read the image
			Mat image = new Mat();
			camera.vcap.read(image);
			//find the center
			Center center = camera.findOneRetroTarget(image);
		}
		else{
			SmartDashboard.putBoolean("Checking For Vision?", false);
		}
		
		//INTAKE ----- toggle
		if (oi.operator.toggleIntake() && !prevStateOpTrigger) {
			//shooter.loadBall(DoubleSolenoid.Value.kForward);
			///	new SetPivotCommand(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
			new SetIntakePosition(intakeState).start();
			intakeState = !intakeState;
		} else {
		//	shooter.loadBall(DoubleSolenoid.Value.kReverse);

		}
		
		//shoot if ready
		if (oi.operator.shoot() && !prevStateShootButton){
			if (shooter.loader.get() == DoubleSolenoid.Value.kReverse && readyToShoot){
				shooter.loader.set(DoubleSolenoid.Value.kForward);
			}
			else{
				shooter.loader.set(DoubleSolenoid.Value.kReverse);
			}
		}
		
		if(oi.operator.loadBallFlywheels()){
			shooter.setLeftFlywheelPower(-0.50);
			shooter.setRightFlywheelPower(0.50);
		} else if(oi.operator.shootBallFlywheels()) {
			new SetFlywheels(true).start();
		} else {
			new SetFlywheels(false).start();
		}
		//tells us if bang bang works
		readyToShoot = true;//((Math.abs(shooter.getRightFlywheelRPM() - ShooterSubsystem.FLYWHEEL_TARGET_RPM) < ShooterSubsystem.FLYWHEEL_TOLERANCE)
				//&& (Math.abs(shooter.getLeftFlywheelRPM() - ShooterSubsystem.FLYWHEEL_TARGET_RPM) < ShooterSubsystem.FLYWHEEL_TOLERANCE));
		
		//pivot state
		if (oi.operator.pivotShootState() && !prevStatePivotUp){
			new SetPivotState(ShooterPivotSubsystem.PivotPID.SHOOT_STATE).start();
		}
		else if(oi.operator.resetPivot() && !prevStateResetButton) {
			 new ResetPivot().start();
		}
		else if (oi.operator.pivotStoreState() && !prevStatePivotMiddle){
			new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
		}
		
		//intake purging/running
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
		
		if(oi.operator.isManualPivotRest() && !shooterPIDIsRunning) {
			double pivotPower = correctForDeadZone(oi.operator.getManualPower()/2.0);
			
			if (pivotPower > 0 && shooterPivot.pastMax() || pivotPower < 0 && shooterPivot.lowerLimitsTriggered()){
				pivotPower = 0;
			}
			
			new SetPivotPower(pivotPower).start();
		}
		
		
		//PREV STATES
		prevStateOpTrigger = oi.operator.toggleIntake();
		prevStateDriveShifter = oi.driver.isDrivetrainLowGearButtonPressed();
		tempPrevState = oi.operator.loadBallFlywheels();
		prevStateShootButton = oi.operator.shoot();
		prevStatePivotUp = oi.operator.pivotShootState();
		prevStatePivotMiddle = oi.operator.pivotStoreState();
		prevStateResetButton = oi.operator.resetPivot();
		
		
		//********updating subsystem*******//
		
		//shooter hal effect counter
		shooterPivot.updateHalEffect();
		//brake
		if (shooterPivot.motorLeft.get() == 0 && shooterPivot.motorRight.get() == 0){
			if (pivotTimer.get() > MIN_STOPPED_TIME){
				shooterPivot.engageBrake(true);
			}
			//if motor is 0 for the first time, start the timer
			if (!prevStateMotorPowerIs0){
				pivotTimer.reset();
				pivotTimer.start();
			}
			//keep this at the end
			prevStateMotorPowerIs0 = true;
		}
		else{
			shooterPivot.engageBrake(false);
			//keep this at the end
			pivotTimer.reset();
			prevStateMotorPowerIs0 = false;
		}
		
		//LOGGING AND DASHBOARD
		log.add(drivetrain.getLoggingData());
		
		SmartDashboard.putData("Shooter Pivot left", shooterPivot.encoderLeft);
		SmartDashboard.putData("Shooter Pivot Right", shooterPivot.encoderRight);
		
		SmartDashboard.putBoolean("Shooter bumper left", shooterPivot.resetBumperLeft.get());
		SmartDashboard.putBoolean("Shooter bumper right", shooterPivot.resetBumperRight.get());
		
		SmartDashboard.putNumber("Shooter Pivot current right", pdp.getCurrent(0));
		SmartDashboard.putNumber("Shooter pivot current left", pdp.getCurrent(15));
		
		SmartDashboard.putData("DT Encoder Left", drivetrain.leftEncoder);
		SmartDashboard.putData("DT Encoder Right", drivetrain.rightEncoder);
		
		/*
		SmartDashboard.putData("GYRO", drivetrain.gyro);
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());

		SmartDashboard.putData("Left Enc", drivetrain.leftEncoder);
		SmartDashboard.putData("Right Enc", drivetrain.rightEncoder);
		SmartDashboard.putBoolean("current gear", currentGear);
		SmartDashboard.putBoolean("current intake state", intakeState);
*/
		SmartDashboard.putNumber("LEFT FLYWHEEL", shooter.getLeftFlywheelRPM());
		SmartDashboard.putNumber("RIGHT FLYWHEEL", shooter.getRightFlywheelRPM());
		SmartDashboard.putBoolean("Ready to Shoot", readyToShoot);
		SmartDashboard.putString("Current Command", " ");
		
		SmartDashboard.putBoolean("Shooter Hall", shooterPivot.reachedResetLimit());//shooterPivot.resetHalEffect.getDirection());
		//SmartDashboard.putNumber("Shooter Hall Effect", shooterPivot.resetHalEffect.get());//shooterPivot.resetHalEffect.getDirection());

		SmartDashboard.putBoolean("Is shooter PID running" , shooterPIDIsRunning);
		
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
