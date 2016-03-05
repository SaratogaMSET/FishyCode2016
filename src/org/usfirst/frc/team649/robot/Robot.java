package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.util.Center;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commandgroups.SemiAutoLoadBall;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.EndSemiAuto;
import org.usfirst.frc.team649.robot.commands.MatchAutoDrive;
import org.usfirst.frc.team649.robot.commands.SetCameraServo;
import org.usfirst.frc.team649.robot.commands.autonomous.AutoCrossChevalDeFrise;
import org.usfirst.frc.team649.robot.commands.intakecommands.RunAllRollers;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakeSpeed;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPower;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.shootercommands.BangBangFlywheels;
import org.usfirst.frc.team649.robot.shootercommands.SetFlywheels;
import org.usfirst.frc.team649.robot.shootercommands.ShooterSet;
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
	public static String ip = "169.254.110.201";
	
	public ArrayList<ArrayList<Double>> log;
	public static Timer timer, pivotTimer;
	public DoubleSolenoid ds;
	public static PowerDistributionPanel pdp;

	SendableChooser chooser;
	
	public static double DEAD_ZONE_TOLERANCE = 0.043;
	public static double MIN_STOPPED_TIME = 0.25; //seconds
	
	//true = down, false = up
	public static boolean intakeState;
	public static boolean shooterState = false;
	//true = high gear, false = low gear
	public static boolean currentGear = true;
	public static boolean shooterPIDIsRunning = false;
	public static boolean readyToShoot = false;
	
	public static boolean isShooting = false;
	
	public static boolean allIntakeRunning = false;
	public static boolean flywheelShootRunning = false;
	public static boolean semiAutoIsRunning = false;
	//prevStates
	public boolean prevStateIntakeUp;
	public boolean prevStateIntakeDeployed;
	public boolean prevStateDriveShifter;
	public boolean prevStateShootButton;
	
	public boolean prevStateMotorPowerIs0; //for brake on pivot
	public boolean prevStatePivotUp;
	public boolean prevStatePivotDown;
	public boolean prevStateResetSafety;

	public boolean prevStateSemiAutoIntake;

	public static boolean isPIDActive;
	
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivetrain = new DrivetrainSubsystem();
		intake = new IntakeSubsystem();
		shooterPivot = new ShooterPivotSubsystem();
		shooter = new ShooterSubsystem();
		camera = new CameraSubsystem(ip);
		isPIDActive= false;
		
		if (Robot.camera.noOpencvErrors){
			if (camera.vcap.isOpened()){
				System.out.println("R-INIT FINISHED VCAP, VALID IP -:- STREAM OPENED");
			}
		}
//		
		log = new ArrayList<>();
		timer = new Timer();
		pivotTimer = new Timer();
		pdp = new PowerDistributionPanel();
		
		prevStateIntakeUp = false;
		prevStateIntakeDeployed = false;
		prevStateDriveShifter = false;
		prevStateMotorPowerIs0 = true;
		prevStateShootButton = false;
		prevStatePivotUp = false;
		prevStatePivotDown = false;
		prevStateResetSafety = false;
		prevStateSemiAutoIntake = false;
		
	}

	public void disabledInit() {
		System.out.println("STARTING LOG: Time, MotorLeft, MotorRight ");
//		for (int i = 0; i < log.size(); i++) {
//			ArrayList<Double> d = log.get(i);
//			System.out.println("BEGINNING_TAG " + d.get(0) + ", " + d.get(1)
//					+ ", " + d.get(2) + ", " + d.get(3) + ", " + d.get(4)
//					+ ", " + d.get(4) + " ENDING_TAG"); //change from 4 to 5 when gyro works
//		}
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	
	public void autonomousInit() {
		drivetrain.resetEncoders();
		drivetrain.gyro.reset();
		shooterPivot.resetEncoders();
		new SetPivotState(5).start();
		new DriveForwardRotate(0, 0).start();
		intakeState = intake.isIntakeDeployed();
		currentGear = drivetrain.driveSol.get() == Value.kForward;
		drivetrain.shift(currentGear);
		new SetIntakePosition(intakeState);
		new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();;
		
//		new DriveForwardRotate(0, 0).start();
//		new MatchAutoDrive(AutonomousSequences.fromPos1, 1).start();;
		//new ResetPivot().start();;
		//new AutoCrossChevalDeFrise().start();;
		
		new BangBangFlywheels(false).start();
		
		shooterPivot.currentPivotState = -1;
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
		SmartDashboard.putData("Shooter Pivot left", shooterPivot.encoderLeft);
		SmartDashboard.putData("Shooter Pivot Right", shooterPivot.encoderRight);
		
		SmartDashboard.putBoolean("Shooter bumper left", !shooterPivot.resetBumperLeft.get());
		SmartDashboard.putBoolean("Shooter bumper right", !shooterPivot.resetBumperRight.get());
		
		SmartDashboard.putBoolean("is flywheel command running", flywheelShootRunning);
		SmartDashboard.putBoolean("is all rollers command running", allIntakeRunning);
		
		SmartDashboard.putBoolean("IR Tripped", Robot.shooter.infraredSensor.get());
		
		SmartDashboard.putNumber("Shooter Pivot current right", pdp.getCurrent(0));
		SmartDashboard.putNumber("Shooter pivot current left", pdp.getCurrent(15));
		
		SmartDashboard.putData("DT Encoder Left", drivetrain.leftEncoder);
		SmartDashboard.putData("DT Encoder Right", drivetrain.rightEncoder);
		
		SmartDashboard.putBoolean("Semi Auto Enabled", semiAutoIsRunning);
		
		//SmartDashboard.putNumber("Cam Servo Angle", camera.camServo.getAngle());
		
		
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());

		SmartDashboard.putString("current Gear", currentGear ? "HIGH GEAR" : "LOW GEAR");
		SmartDashboard.putString("current Intake State", intakeState ? "DEPLOYED" : "UP");
		
		SmartDashboard.putBoolean("absolute intake state HAL", intake.isIntakeDeployed());

		SmartDashboard.putNumber("LEFT FLYWHEEL", shooter.getLeftFlywheelRPM());
		SmartDashboard.putNumber("RIGHT FLYWHEEL", shooter.getRightFlywheelRPM());
		SmartDashboard.putBoolean("Ready to Shoot", readyToShoot);
		
		SmartDashboard.putBoolean("IS SHOOTING", isShooting);
		SmartDashboard.putString("Current Command", " ");
		
		SmartDashboard.putBoolean("Shooter Hall", shooterPivot.reachedResetLimit());//shooterPivot.resetHalEffect.getDirection());
		//SmartDashboard.putNumber("Shooter Hall Effect", shooterPivot.resetHalEffect.get());//shooterPivot.resetHalEffect.getDirection());

		SmartDashboard.putBoolean("Is shooter PID running" , shooterPIDIsRunning);
		SmartDashboard.putNumber("Current Servo", camera.camServo.getAngle());
	}

	public void teleopInit() {
		timer.reset();
		timer.start();
		pivotTimer.reset();
		log = new ArrayList<>();
		//drivetrain.gyro.reset();
		drivetrain.resetEncoders();
		drivetrain.gyro.reset();
		shooterPivot.resetEncoders();
		new SetPivotState(5).start();
		new DriveForwardRotate(0, 0).start();
		intakeState = intake.isIntakeDeployed();
		currentGear = drivetrain.driveSol.get() == Value.kForward;
		drivetrain.shift(currentGear);
		new SetIntakePosition(intakeState);
		//new SetCameraServo(CameraSubsystem.CAM_UP_ANGLE).start();
		new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();;
		
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
		//camera.vcap.open("http://axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");

		shooterPivot.currentPivotState = -1;
	}

	public void teleopPeriodic() {
		//these may or may not be overriden by commands
		SmartDashboard.putString("DT Current Command", " ");
		SmartDashboard.putString("SHOOTER Current Command", " ");
		SmartDashboard.putString("SHOOTERPIVOT Current Command", " ");
		SmartDashboard.putString("INTAKE Current Command", " ");
		
		Scheduler.getInstance().run();
		
		SmartDashboard.putBoolean("INTAKE HAL", intake.hal.get());
		//new DriveForwardRotate(correctForDeadZone(oi.driver.getForward()), correctForDeadZone(oi.driver.getRotation())).start();
		new DriveForwardRotate(oi.driver.getForward(), oi.driver.getRotation()).start();
		
		//CAMERA
//		if (camera.vcap.isOpened()){
//			SmartDashboard.putBoolean("Checking For Vision?", true);
//			//read the image
//			
//			Mat image = new Mat();
//			camera.vcap.read(image);
//			//find the center
//			//Center center = camera.findOneRetroTarget(image); TODO re put this in eventually
//		
//		}
//		else{
//			SmartDashboard.putBoolean("Checking For Vision?", false);
//		}
		
//		if (readyToShoot){
//			if (oi.operator.shoot() && !prevStateShootButton){
//				new ShooterSet(shooterState? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward).start();
//				shooterState = !shooterState;
//			}
//		}
		
		if (readyToShoot && oi.operator.shootBallFlywheels() && oi.operator.shoot() && !prevStateShootButton){
			new ShootTheShooter().start();
		}
		
		//INTAKE ----- toggle
//		if (oi.operator.toggleIntake() && !prevStateOpTrigger) {
//			//shooter.loadBall(DoubleSolenoid.Value.kForward);
//			///	new SetPivotCommand(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
//			new SetIntakePosition(!intakeState).start();
//			//intakeState = !intakeState; //THIS NOW HAPPENS WITHIN THE COMMAND
//		} else {
//		//	shooter.loadBall(DoubleSolenoid.Value.kReverse);
//
//		}
		
		//INTAKE ------2 button
		if (oi.operator.intakeUp() && !prevStateIntakeUp){
			new SetIntakePosition(IntakeSubsystem.UP).start();
		}
		if (oi.operator.intakeDeploy() && !prevStateIntakeDeployed){
			new SetIntakePosition(!IntakeSubsystem.UP).start();
		}
		
		//CAMERA SERVO
		
		
		//move up
		if (oi.driver.isCameraUpPressed()){
			new SetCameraServo(CameraSubsystem.CAM_UP_ANGLE).start();
		}
		else{
			new SetCameraServo(CameraSubsystem.CAM_DOWN_ANGLE).start();
		}
		
		
		
		if (oi.operator.isSemiAutonomousIntakePressed() && !prevStateSemiAutoIntake){
			semiAutoIsRunning = true;
			new SemiAutoLoadBall().start();
		}
		
		
		//pivot state
//		if (oi.operator.pivotShootState() && !prevStatePivotUp){
//			new SetPivotState(ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE).start();
//		}
//		else if(oi.operator.resetPivot() && !prevStateResetButton) {
//			 new ResetPivot().start();
//		}
//		else if (oi.operator.pivotStoreState() && !prevStatePivotMiddle){
//			new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
//		}
		
		//PIVOT
		
		//if going up
		if (oi.operator.pivotMoveUp() && !prevStatePivotUp){
			int nextRegion = shooterPivot.getClosestNextSetpointState(true);
			new SetPivotState(nextRegion).start();
		}
		//if going down
		else if(oi.operator.pivotMoveDown() && !prevStatePivotDown) {
			int nextRegion = shooterPivot.getClosestNextSetpointState(false);
			if (nextRegion == ShooterPivotSubsystem.PivotPID.PICKUP_STATE){
				//if we are trying to reset (with safety button) or if the next state is the reset state
				new ResetPivot().start();
			}
			else{
				//if going down to any other region, use the regular pivot command
				new SetPivotState(nextRegion).start();
			}
		}
		else if (oi.operator.pivotResetSafety() && !prevStateResetSafety){
			new ResetPivot().start();
		}
		
		if (!semiAutoIsRunning){
//		
			//intake purging/running
			if(oi.operator.purgeIntake()) {
				//new SetIntakeSpeed(IntakeSubsystem.FORWARD_ROLLER_PURGE_SPEED, IntakeSubsystem.CENTERING_MODULE_PURGE_SPEED).start(); 
				new RunAllRollers(ShooterSubsystem.OUT, !ShooterSubsystem.UNTIL_IR).start();
				allIntakeRunning = true;
			} else if (oi.operator.runIntake()) {
				//new SetIntakeSpeed(IntakeSubsystem.FORWARD_ROLLER_INTAKE_SPEED, IntakeSubsystem.CENTERING_MODULE_INTAKE_SPEED).start();
				new SemiAutoLoadBall().start();
				allIntakeRunning = true;
			} else {
				//just the intakes off here to avoid conflicts
				new SetIntakeSpeed(IntakeSubsystem.INTAKE_OFF_SPEED, IntakeSubsystem.INTAKE_OFF_SPEED).start();
				new EndSemiAuto(true).start();
				//new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();
				allIntakeRunning = false;
			}
			
			//flywheel control
			if (!allIntakeRunning){
//				if(oi.operator.loadBallFlywheels()){
//					new SetFlywheels(ShooterSubsystem.FLYWHEEL_INTAKE_POWER, -ShooterSubsystem.FLYWHEEL_INTAKE_POWER).start();;
//					flywheelShootRunning = true;
//				} else 
				if(oi.operator.shootBallFlywheels()) {
					//new SetFlywheels(1.0, -1.0).start();
					new BangBangFlywheels(true).start();
					flywheelShootRunning = true;
				} else {
					//new SetFlywheels(0, 0).start();//BangBangFlywheels(false).start();
					flywheelShootRunning = false;
				}
			}
			
			if (!allIntakeRunning && !flywheelShootRunning){
				new SetFlywheels(0, 0).start();
			}
		}
		//tells us if bang bang works
		readyToShoot = ((Math.abs(shooter.getRightFlywheelRPM() - ShooterSubsystem.FLYWHEEL_TARGET_RPM) < ShooterSubsystem.FLYWHEEL_TOLERANCE)
				&& (Math.abs(shooter.getLeftFlywheelRPM() - ShooterSubsystem.FLYWHEEL_TARGET_RPM) < ShooterSubsystem.FLYWHEEL_TOLERANCE));
		
		
		if(oi.driver.isDrivetrainLowGearButtonPressed() && !prevStateDriveShifter){
			drivetrain.shift(!currentGear);
			currentGear = !currentGear;
		} else {
			drivetrain.shift(currentGear);
		}
		
		if (!shooterPIDIsRunning){
			if(oi.operator.isManualPivotReset()) {
				double pivotPower = correctForDeadZone(oi.operator.getManualPower()/2.0);
				
				if (pivotPower > 0 && shooterPivot.pastMax() || pivotPower < 0 && shooterPivot.lowerLimitsTriggered()){
					pivotPower = 0;
				}
				
				new SetPivotPower(pivotPower).start();
			}
			else{
				new SetPivotPower(0).start();
			}
		}
		
		
		//PREV STATES
		prevStateIntakeUp = oi.operator.intakeUp();
		prevStateIntakeDeployed = oi.operator.intakeDeploy();
		prevStateDriveShifter = oi.driver.isDrivetrainLowGearButtonPressed();
		prevStateShootButton = oi.operator.shoot();
		
		prevStatePivotUp = oi.operator.pivotMoveUp();
		prevStatePivotDown = oi.operator.pivotMoveDown();
		prevStateResetSafety = oi.operator.pivotResetSafety();
		
		prevStateSemiAutoIntake = oi.operator.isSemiAutonomousIntakePressed();
		
		
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
		
		SmartDashboard.putBoolean("Shooter bumper left", !shooterPivot.resetBumperLeft.get());
		SmartDashboard.putBoolean("Shooter bumper right", !shooterPivot.resetBumperRight.get());
		
		SmartDashboard.putBoolean("is flywheel command running", flywheelShootRunning);
		SmartDashboard.putBoolean("is all rollers command running", allIntakeRunning);
		
		SmartDashboard.putBoolean("IR Tripped", Robot.shooter.infraredSensor.get());
		
		SmartDashboard.putNumber("Shooter Pivot current right", pdp.getCurrent(0));
		SmartDashboard.putNumber("Shooter pivot current left", pdp.getCurrent(15));
		
		SmartDashboard.putData("DT Encoder Left", drivetrain.leftEncoder);
		SmartDashboard.putData("DT Encoder Right", drivetrain.rightEncoder);
		
		SmartDashboard.putBoolean("Semi Auto Enabled", semiAutoIsRunning);
		
		//SmartDashboard.putNumber("Cam Servo Angle", camera.camServo.getAngle());
		
		
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());
/*
		SmartDashboard.putData("Left Enc", drivetrain.leftEncoder);
		SmartDashboard.putData("Right Enc", drivetrain.rightEncoder);*/
		SmartDashboard.putString("current Gear", currentGear ? "HIGH GEAR" : "LOW GEAR");
		SmartDashboard.putString("current Intake State", intakeState ? "DEPLOYED" : "UP");
		
		SmartDashboard.putBoolean("absolute intake state HAL", intake.isIntakeDeployed());

		SmartDashboard.putNumber("LEFT FLYWHEEL", shooter.getLeftFlywheelRPM());
		SmartDashboard.putNumber("RIGHT FLYWHEEL", shooter.getRightFlywheelRPM());
		SmartDashboard.putBoolean("Ready to Shoot", readyToShoot);
		
		SmartDashboard.putBoolean("IS SHOOTING", isShooting);
		
		
		
		SmartDashboard.putNumber("NEXT STATE on PIVOT (UP)", shooterPivot.getClosestNextSetpointState(true));
		SmartDashboard.putNumber("NEXT STATE on PIVOT (DOWN)", shooterPivot.getClosestNextSetpointState(false));
		SmartDashboard.putNumber("Current Pivot State", shooterPivot.currentPivotState);
		
		SmartDashboard.putBoolean("Shooter Hall", shooterPivot.reachedResetLimit());//shooterPivot.resetHalEffect.getDirection());
		//SmartDashboard.putNumber("Shooter Hall Effect", shooterPivot.resetHalEffect.get());//shooterPivot.resetHalEffect.getDirection());

		SmartDashboard.putBoolean("Is shooter PID running" , shooterPIDIsRunning);
		SmartDashboard.putNumber("Current Servo", camera.camServo.getAngle());
		
	}
	
	public double correctForDeadZone(double joyVal){
		   return Math.abs(joyVal) >= DEAD_ZONE_TOLERANCE ? joyVal : 0;
   }
	
	public boolean isStalling(Victor motor, int pdpPort){
		return false;
	}
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
