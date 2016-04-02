package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.Compressor;
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

import org.opencv.core.*;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem.PivotPID;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.LeftDTPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.RightDTPID;
import org.usfirst.frc.team649.robot.util.Center;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.usfirst.frc.team649.robot.RobotMap.ShooterPivot;
import org.usfirst.frc.team649.robot.commandgroups.SemiAutoLoadBall;
import org.usfirst.frc.team649.robot.commandgroups.SetDefenseMode;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivePIDLeft;
import org.usfirst.frc.team649.robot.commands.DrivePIDRight;
import org.usfirst.frc.team649.robot.commands.MatchAutoDrive;
import org.usfirst.frc.team649.robot.commands.SetCameraPiston;
import org.usfirst.frc.team649.robot.commands.TurnWithEncoders;
import org.usfirst.frc.team649.robot.commands.TurnWithGyro;
import org.usfirst.frc.team649.robot.commands.VisionLoop;
import org.usfirst.frc.team649.robot.commands.autonomous.AutoCrossChevalDeFrise;
import org.usfirst.frc.team649.robot.commands.autonomous.AutoTwoBallLowBar;
import org.usfirst.frc.team649.robot.commands.intakecommands.RunAllRollers;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakePosition;
import org.usfirst.frc.team649.robot.commands.intakecommands.SetIntakeSpeed;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.EngageBrakes;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.ResetPivot;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotPower;
import org.usfirst.frc.team649.robot.commands.shooterpivotcommands.SetPivotState;
import org.usfirst.frc.team649.robot.runnables.EndAppThread;
import org.usfirst.frc.team649.robot.runnables.InitializeAdbThread;
import org.usfirst.frc.team649.robot.runnables.PullVisionTxtThread;
import org.usfirst.frc.team649.robot.runnables.SystemCheckThread;
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

	//SUBSYSTEMS
	public static OI oi;
	public static DrivetrainSubsystem drivetrain;
	public static LeftDTPID leftDT;
	public static RightDTPID rightDT;
	public static IntakeSubsystem intake;
	public static ShooterPivotSubsystem shooterPivot;
	public static ShooterSubsystem shooter;
	public static CameraSubsystem camera;
	public static PowerDistributionPanel pdp;
	public static Compressor c; 
	
	//OI
	public static final double DEAD_ZONE_TOLERANCE = 0.043;
	boolean manual;
	public static boolean isManualPressed;
	
	//DT	
	public static boolean isPIDActive;
	public static boolean isPIDActiveLeft;
	public static boolean isPIDActiveRight;
	//true = high gear, false = low gear
	public static boolean currentGear = true;
	static boolean cameraUp = false;
	
	//PIVOT
	public static Timer pivotTimer;
	public static final double MIN_STOPPED_TIME = 0.15; //seconds
	public static boolean shooterPIDIsRunning = false;
	
	//INTAKE
	public static boolean allIntakeRunning = false;
	//true = down, false = up
	public static boolean intakeState;
	
	//SHOOTER
	public static boolean readyToShoot = false;
	public static boolean flywheelShootRunning = false;
	public static boolean isShooting = false;
	
	//VISION
	public static String ip = "N/A";
	public static String initPath = "/home/admin/initializeAdb.sh";
	public static String pullPath = "/home/admin/pullTextFile.sh";
	public static String endPath = "/home/admin/endApp.sh";
	public static String visionFile = "/home/admin/vision.txt";
	public static int PULL_PERIOD = 20;
	public static VisionLoop visionCommand;
	public static Center currCenter;
	public static boolean runVision;
	public static ScheduledExecutorService adbTimer;
	
	//LOGGING
	public ArrayList<ArrayList<Double>> log;
	public static Timer timer;
	
	//MISC
	public static boolean semiAutoIsRunning = false; //semi auto indicator
	public static boolean robotEnabled = false; //indicator
	public SendableChooser chooser; //autonomous selector
	public Thread systemCheck; //thread stuff
	
	//PREV STATES
	public boolean prevStateIntakeUp;
	public boolean prevStateIntakeDeployed;
	public boolean prevStateDriveShifter;
	public boolean prevStateShootButton;
	
	public boolean prevStateMotorPowerIs0; //for brake on pivot
	public boolean prevStatePivotCloseShot;
	public boolean prevStatePivotFarShot;
	public boolean prevStateStore;
	public boolean prevStateResetSafety;
	
	public boolean prevStateDefenseMode;
	public boolean prevStateSemiAutoIntake;
	
	public boolean prevStateManualFirePiston;
	public boolean prevStateCameraUp;
	
	///////**********************ROBOT INIT************************//////
	
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivetrain = new DrivetrainSubsystem();
		leftDT = new LeftDTPID();
		rightDT = new RightDTPID();
		intake = new IntakeSubsystem();
		shooterPivot = new ShooterPivotSubsystem();
		shooter = new ShooterSubsystem();
		camera = new CameraSubsystem(ip);
		isPIDActive= false;
		runVision = false;
		
		log = new ArrayList<>();
		timer = new Timer();
		pivotTimer = new Timer();
		pdp = new PowerDistributionPanel();
		
		prevStateIntakeUp = false;
		prevStateIntakeDeployed = false;
		prevStateDriveShifter = false;
		prevStateMotorPowerIs0 = true;
		prevStateShootButton = false;
		prevStatePivotCloseShot = false;
		prevStatePivotFarShot = false;
		prevStateStore = false;
		prevStateResetSafety = false;
		prevStateSemiAutoIntake = false;
		prevStateDefenseMode = false;
		prevStateManualFirePiston = false;
		prevStateCameraUp = false;
		
		isManualPressed = false;
		
		robotEnabled = false;
		
		currCenter = new Center(-1,-1);
	}
	
	/////////////****************AUTONOMOUS***************//////////////
	
	public void autonomousInit() {
		drivetrain.resetEncoders();
		drivetrain.gyro.reset();
		shooterPivot.resetEncoders();
		new SetPivotState(ShooterPivotSubsystem.PivotPID.CURRENT_STATE).start();
		new DriveForwardRotate(0, 0).start();
		new DrivePIDRight(0).start();
		new DrivePIDLeft(0).start();
		intakeState = intake.isIntakeDeployed();
		currentGear = drivetrain.driveSol.get() == Value.kForward;
		drivetrain.shift(currentGear);
		new SetIntakePosition(intakeState);
		new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();;
		
		//runVision = true;
		//visionLoop.start();
		
		
		new DriveForwardRotate(0, 0).start();
														//new AutoCrossChevalDeFrise().start();

		System.out.println(drivetrain.gyro.getAngle());
		shooterPivot.currentPivotState = -1;

		//VERY IMPORTANT
		robotEnabled = true;
		
		//start pulling txt and updating it
		startVisionThreads();
		
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
		if (!shooterPIDIsRunning && shooterPivot.motorLeft.get() < DEAD_ZONE_TOLERANCE && shooterPivot.motorRight.get() < DEAD_ZONE_TOLERANCE){
			
			if (pivotTimer.get() > MIN_STOPPED_TIME){
				new EngageBrakes().start();
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
	
		logAndDashboard(); //TODO put back in
		//update the flywheel speed constants
		shooter.updateShooterConstants();
	}

	/////////////****************TELEOP***************//////////////
	
	public void teleopInit() {
		timer.reset();
		timer.start();
		pivotTimer.reset();
		log = new ArrayList<>();
		//drivetrain.gyro.reset();
		drivetrain.resetEncoders();
		drivetrain.gyro.reset();
		new DrivePIDRight(0).start();
		new DrivePIDLeft(0).start();
		shooterPivot.resetEncoders();
		new SetPivotState(ShooterPivotSubsystem.PivotPID.CURRENT_STATE).start();
		new DriveForwardRotate(0, 0).start();
		intakeState = intake.isIntakeDeployed();
		currentGear = drivetrain.driveSol.get() == Value.kForward;
		drivetrain.shift(currentGear);
		new SetIntakePosition(intakeState);
		new SetCameraPiston(!CameraSubsystem.CAM_UP).start();
		new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();;
		

		shooterPivot.currentPivotState = -1;
		
		//VERY IMPORTANT
		robotEnabled = true;
		
		//start pulling txt and updating it
		startVisionThreads();	
	}

	public void teleopPeriodic() {
		//these may or may not be overriden by commands
		SmartDashboard.putString("DT Current Command", " ");
		SmartDashboard.putString("SHOOTER Current Command", " ");
		SmartDashboard.putString("SHOOTERPIVOT Current Command", " ");
		SmartDashboard.putString("INTAKE Current Command", " ");
		
		Scheduler.getInstance().run();
		
		new DriveForwardRotate(correctForDeadZone(oi.driver.getForward()), correctForDeadZone(oi.driver.getRotation())).start();
		//new DriveForwardRotate(oi.driver.getForward(), oi.driver.getRotation()).start();
		
		if (readyToShoot && oi.operator.shootBallFlywheels() && oi.operator.shoot() && !prevStateShootButton){
			new ShootTheShooter().start();
		}
		
		
		//INTAKE ------2 button
		if (oi.operator.intakeUp() && !prevStateIntakeUp){
			new SetIntakePosition(IntakeSubsystem.UP).start();
		}
		if (oi.operator.intakeDeploy() && !prevStateIntakeDeployed){
			new SetIntakePosition(!IntakeSubsystem.UP).start();
		}
		
		//CAMERA PISTON
		
		//move up
//		if (oi.driver.isCameraUpPressed()){
//			new SetCameraPiston(CameraSubsystem.CAM_UP).start();
//		}
//		else{
//			new SetCameraPiston(!CameraSubsystem.CAM_UP).start();
//		}
		
		if (oi.driver.isCameraUpPressed() && !prevStateCameraUp){
			new SetCameraPiston(cameraUp ? !CameraSubsystem.CAM_UP : CameraSubsystem.CAM_UP).start();
			cameraUp = !cameraUp;
		}
		else{
			new SetCameraPiston(cameraUp ? CameraSubsystem.CAM_UP : !CameraSubsystem.CAM_UP).start();
		}
		
		
		if (oi.operator.isSemiAutonomousIntakePressed() && !prevStateSemiAutoIntake){
			new SemiAutoLoadBall().start();
		}
//		
		
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
		if (oi.operator.pivotCloseShot() && !prevStatePivotCloseShot){
			new SetPivotState(ShooterPivotSubsystem.PivotPID.CLOSE_SHOOT_STATE).start();		
		}
		//if going down
		else if(oi.operator.pivotFarShot() && !prevStatePivotFarShot) {
			new SetPivotState(ShooterPivotSubsystem.PivotPID.FAR_SHOOT_STATE).start();
		}
		else if (oi.operator.pivotStoreState()&& !prevStateStore){
			new SetPivotState(ShooterPivotSubsystem.PivotPID.STORING_STATE).start();
		}
		else if (oi.operator.pivotReset() && !prevStateResetSafety){
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
				new RunAllRollers(ShooterSubsystem.IN, ShooterSubsystem.UNTIL_IR).start();
//				new SemiAutoLoadBall().start();
				allIntakeRunning = true;
			} else {
				//just the intakes off here to avoid conflicts
				new SetIntakeSpeed(IntakeSubsystem.INTAKE_OFF_SPEED, IntakeSubsystem.INTAKE_OFF_SPEED).start();
				//new EndSemiAuto(true).start();
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
					//new SetFlywheels(0.7, -0.7).start();
					new BangBangFlywheels(true).start();
					flywheelShootRunning = true;
				} else {
					//new SetFlywheels(0, 0).start();//BangBangFlywheels(false).start();
					flywheelShootRunning = false;
				}
			}
			
//			if(oi.operator.shoot()){
//				new ShooterSet(DoubleSolenoid.Value.kForward).start();
//			} else {
//				new ShooterSet(DoubleSolenoid.Value.kReverse).start();
//			}
//			
			if (!allIntakeRunning && !flywheelShootRunning){
				new SetFlywheels(0, 0).start();
			}
			
			//DEFENSE STATE
			if (oi.operator.isDefenseState() && !prevStateDefenseMode){
				new SetDefenseMode().start();
			}
		}
		//tells us if bang bang works
		readyToShoot = ((Math.abs(shooter.getRightFlywheelRPM() - ShooterSubsystem.flywheelTargetRPM) < ShooterSubsystem.flywheelTolerance)
				&& (Math.abs(shooter.getLeftFlywheelRPM() - ShooterSubsystem.flywheelTargetRPM) < ShooterSubsystem.flywheelTolerance));
		
		
		if (oi.operator.isManualFirePiston() && !prevStateManualFirePiston){
			new ShootTheShooter().start();
		}
		
		
		if(oi.driver.isDrivetrainLowGearButtonPressed()){
			drivetrain.shift(DrivetrainSubsystem.HIGH_GEAR);
			currentGear = DrivetrainSubsystem.HIGH_GEAR;
		} else {
			drivetrain.shift(!DrivetrainSubsystem.HIGH_GEAR);
			currentGear = !DrivetrainSubsystem.HIGH_GEAR;
		}
		
		isManualPressed = oi.operator.isManualOverrideOperator();
		
		if (!shooterPIDIsRunning){
			if(isManualPressed) {
				double pivotPower = oi.operator.getManualPower()/2.0;
				//shooterPivot.engageBrake(false);
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
		
		prevStatePivotCloseShot = oi.operator.pivotCloseShot();
		prevStatePivotFarShot = oi.operator.pivotFarShot();
		prevStateStore = oi.operator.pivotStoreState();
		prevStateResetSafety = oi.operator.pivotReset();
		
		prevStateSemiAutoIntake = oi.operator.isSemiAutonomousIntakePressed();
		prevStateDefenseMode = oi.operator.isDefenseState();
		
		prevStateManualFirePiston = oi.operator.isManualFirePiston();
		prevStateCameraUp = oi.driver.isCameraUpPressed();
		
		
		//********updating subsystem*******//
		
		//shooter hal effect counter
		//shooterPivot.updateHalEffect();
		
		//update the flywheel speed constants
		shooter.updateShooterConstants();
		
		//brake
		if (!isManualPressed){ //if manual override isn't running
			if (!shooterPIDIsRunning && shooterPivot.motorLeft.get() < DEAD_ZONE_TOLERANCE && shooterPivot.motorRight.get() < DEAD_ZONE_TOLERANCE){
				
				if (pivotTimer.get() > MIN_STOPPED_TIME){
					new EngageBrakes().start();
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
		}
		else{
			shooterPivot.engageBrake(false);
			pivotTimer.reset();
		}
		//System.out.println(shooterPivot.motorLeft.get());
		
		//LOGGING AND DASHBOARD
		logAndDashboard();
	}
	
	
	/////////////****************TEST***************//////////////
	
	public void testInit(){
		timer.reset();
		timer.start();
		pivotTimer.reset();
		log = new ArrayList<>();
		//drivetrain.gyro.reset();
		drivetrain.resetEncoders();
		drivetrain.gyro.reset();
		new DrivePIDRight(0).start();
		new DrivePIDLeft(0).start();
		shooterPivot.resetEncoders();
		new SetPivotState(ShooterPivotSubsystem.PivotPID.CURRENT_STATE).start();
		new DriveForwardRotate(0, 0).start();
		intakeState = intake.isIntakeDeployed();
		currentGear = drivetrain.driveSol.get() == Value.kForward;
		drivetrain.shift(currentGear);
		new SetIntakePosition(intakeState);
		new SetCameraPiston(!CameraSubsystem.CAM_UP).start();
		new RunAllRollers(ShooterSubsystem.OFF, !ShooterSubsystem.UNTIL_IR).start();;

		//VERY IMPORTANT
		robotEnabled = true;
		
		System.out.println("TEST MODE");
		systemCheck = new Thread(new SystemCheckThread());
		systemCheck.start();
		
	}
	
	public void testPeriodic() {
		//LiveWindow.run();
		Scheduler.getInstance().run();

	}
	
	
	/////////////****************DISABLED***************//////////////

	public void disabledInit() {
		System.out.println("STARTING LOG: Time, MotorLeft, MotorRight ");
		//saveLog();
		
		//stop reading from file
		
		
		//SUPER IMPORTANT
		robotEnabled = false;

		if (systemCheck != null){
			System.out.println("ENDING VISION");
			systemCheck.interrupt();
		}
		
		//end vision
		endVisionThreads();
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
	}

	
	
	//////////////***************MISC***************//////////////
	
	public void saveLog(){
		if (log != null){
			for (int i = 0; i < log.size(); i++) {
				ArrayList<Double> d = log.get(i);
				System.out.println("BEGINNING_TAG " + d.get(0) + ", " + d.get(1)
						+ ", " + d.get(2) + ", " + d.get(3) + ", " + d.get(4)
						+ ", " + d.get(4) + " ENDING_TAG"); //change from 4 to 5 when gyro works
			}
			System.out.println("FINISHED PRINT");
		}
		else {
			System.out.println("PRINT FAILED: Log is null");
		}
	}
	
	public void startVisionThreads(){
		System.out.println("STARTING VISION");

		Thread init = new Thread(new InitializeAdbThread());
		init.start();
		
		//wait for thread to die
		try {
			init.join();
			System.out.println(">--VISION initialized --ROBOT");
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("X--VISION initialize FAILED --ROBOT");
		}
		
		this.adbTimer = Executors.newSingleThreadScheduledExecutor();
		this.adbTimer.scheduleAtFixedRate(new Thread(new PullVisionTxtThread()), 0, PULL_PERIOD, TimeUnit.MILLISECONDS);
	}
	
	public void endVisionThreads(){
		if (adbTimer != null){
			this.adbTimer.shutdown();
			try {
				this.adbTimer.awaitTermination(33, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
		
		Thread end = new Thread(new EndAppThread());
		end.start();
		try {
			end.join();
			System.out.println("App CLOSED --ROBOT");
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	
	//////SUPER IMPORTANT///////
	public void logAndDashboard(){
		log.add(drivetrain.getLoggingData());
		
		SmartDashboard.putString("CENTER OF VISION", "(" + currCenter.x + ", " + currCenter.y + ")");
		
		SmartDashboard.putData("Shooter Pivot left", shooterPivot.encoderLeft);
		SmartDashboard.putData("Shooter Pivot Right", shooterPivot.encoderRight);
		
		SmartDashboard.putBoolean("Shooter bumper left", !shooterPivot.resetBumperLeft.get());
		SmartDashboard.putBoolean("Shooter bumper right", !shooterPivot.resetBumperRight.get());
		
		SmartDashboard.putBoolean("is flywheel command running", flywheelShootRunning);
		SmartDashboard.putBoolean("is all rollers command running", allIntakeRunning);
		
		SmartDashboard.putBoolean("No ball(IR)", Robot.shooter.infraredSensor.get());
		
		SmartDashboard.putNumber("Shooter Pivot current right", pdp.getCurrent(0));
		SmartDashboard.putNumber("Shooter pivot current left", pdp.getCurrent(15));
		
		SmartDashboard.putData("DT Encoder Left", drivetrain.leftEncoder);
		SmartDashboard.putData("DT Encoder Right", drivetrain.rightEncoder);
		
		SmartDashboard.putBoolean("Semi Auto Enabled", semiAutoIsRunning);
		
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());

		SmartDashboard.putString("current Gear", currentGear ? "HIGH GEAR" : "LOW GEAR");
		SmartDashboard.putString("current Intake State", intakeState ? "DEPLOYED" : "UP");
		
		SmartDashboard.putBoolean("absolute intake state HAL", intake.isIntakeDeployed());

		SmartDashboard.putNumber("LEFT FLYWHEEL", shooter.getLeftFlywheelRPM());
		SmartDashboard.putNumber("RIGHT FLYWHEEL", shooter.getRightFlywheelRPM());
		SmartDashboard.putBoolean("Ready to Shoot", readyToShoot);
		
		SmartDashboard.putBoolean("IS SHOOTING", isShooting);

		SmartDashboard.putBoolean("INTAKE HAL", intake.hal.get());
		
		SmartDashboard.putNumber("NEXT STATE on PIVOT (UP)", shooterPivot.getClosestNextSetpointState(true));
		SmartDashboard.putNumber("NEXT STATE on PIVOT (DOWN)", shooterPivot.getClosestNextSetpointState(false));
		SmartDashboard.putNumber("Current Pivot State", shooterPivot.currentPivotState);
		
		SmartDashboard.putBoolean("Shooter Pivot Hal", shooterPivot.isResetHalTripped());//shooterPivot.resetHalEffect.getDirection());
		//SmartDashboard.putNumber("Shooter Hall Effect", shooterPivot.resetHalEffect.get());//shooterPivot.resetHalEffect.getDirection());

		SmartDashboard.putBoolean("Is shooter PID running" , shooterPIDIsRunning);
		SmartDashboard.putNumber("PIVOT ANGLE", shooterPivot.getPivotAngle());
		SmartDashboard.putNumber("CHOSEN PIVOT ANGLE", shooterPivot.getClosestAngleToSetpoint(shooterPivot.getSetpoint()));
		
		SmartDashboard.putNumber("Joy POV", oi.operatorJoystick.getPOV());
		SmartDashboard.putNumber("Left Centering Module Current", pdp.getCurrent(5));
		SmartDashboard.putNumber("Left Centering Module Current", pdp.getCurrent(10));
	}
	
	public double correctForDeadZone(double joyVal){
		   return Math.abs(joyVal) >= DEAD_ZONE_TOLERANCE ? joyVal : 0;
   }
	
	public boolean isStalling(Victor motor, int pdpPort){
		return false;
	}
	
}
