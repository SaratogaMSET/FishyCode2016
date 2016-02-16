package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;

import org.usfirst.frc.team649.robot.RobotMap.Intake;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotateCommand;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterPivotSubsystem;
import org.usfirst.frc.team649.robot.subsystems.ShooterSubsystem;

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
	//public static ShooterSubsystem shooter;
	public ArrayList<ArrayList<Double>> log;
	public static Timer timer;
	public DoubleSolenoid ds;

	SendableChooser chooser;
	
	//true = up, false = down
	public static boolean intakeState = false;
	//true = high gear, false = low gear
	public static boolean currentGear = true;
	
	//prevStates
	public boolean prevStateOpTrigger;
	public boolean prevStateDHorizontalTrigger;
	public boolean prevStateDVerticalTrigger;

	
	public void robotInit() {
		oi = new OI();
		chooser = new SendableChooser();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		drivetrain = new DrivetrainSubsystem();
		intake = new IntakeSubsystem();
		// shooterPivotSubsystem = new ShooterPivotSubsystem();
		log = new ArrayList<>();
		timer = new Timer();
		
		prevStateOpTrigger = false;
		prevStateDHorizontalTrigger = false;
		prevStateDVerticalTrigger = false;
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
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		drivetrain.driveFwdRot(oi.driver.getForward(), oi.driver.getRotation());
		
		//INTAKE ----- toggle
		if (oi.operatorJoystick.getRawButton(1) && !prevStateOpTrigger) {
			intake.setSolenoids(!intakeState);
			intakeState = !intakeState;
		} else {
			intake.setSolenoids(intakeState);
		}
		/*
		if(oi.operatorJoystick.getRawButton(3)) {
			intake.setCenteringModuleSpeed(1.0);
			intake.setFwdRolSpd(1.0);
		} else if (oi.operatorJoystick.getRawButton(4)) {
			intake.setCenteringModuleSpeed(-1.0);
			intake.setFwdRolSpd(-1.0);
		} else {
			intake.setCenteringModuleSpeed(0.0);
			intake.setFwdRolSpd(0.0);
		}
		*/
		if((oi.driveJoystickHorizontal.getRawButton(1) && !prevStateDHorizontalTrigger) 
				|| (oi.driveJoystickVertical.getRawButton(1) && !prevStateDVerticalTrigger)){
			drivetrain.shift(!currentGear);
			currentGear = !currentGear;
		} else {
			drivetrain.shift(currentGear);
		}
		
		/*
		if (oi.operatorJoystick.getRawButton(2)) {
			shooter.runPunch(Value.kForward);
		} else if (oi.operatorJoystick.getRawButton(3)) {
			shooter.runPunch(Value.kReverse);

		} else {
			shooter.runPunch(Value.kOff);
		}
		*/
		log.add(drivetrain.getLoggingData());
		SmartDashboard.putData("GYRO", drivetrain.gyro);
		SmartDashboard.putNumber("GYRO ANGLE", drivetrain.gyro.getAngle());
		
		SmartDashboard.putData("Left Enc", drivetrain.leftEncoder);
		SmartDashboard.putData("Right Enc", drivetrain.rightEncoder);
		SmartDashboard.putBoolean("current gear", currentGear);
		SmartDashboard.putBoolean("current intake state", intakeState);
		//SmartDashboard.putData("", );
		
		//PREV STATES
		prevStateOpTrigger = oi.operatorJoystick.getRawButton(1);
		prevStateDHorizontalTrigger = oi.driveJoystickHorizontal.getRawButton(1);
		prevStateDVerticalTrigger = oi.driveJoystickVertical.getRawButton(1);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}
