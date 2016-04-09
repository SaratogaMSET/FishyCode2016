package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public Joystick operatorJoystick;
	public Joystick driveJoystickHorizontal;
	public Joystick driveJoystickVertical;
	public Joystick manualJoystick;
	public Operator operator;
	public Driver driver;
	public Manual manual;
	
	public OI() {
		operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);
		driveJoystickHorizontal = new Joystick(RobotMap.DRIVE_LEFT_JOYSTICK);
		driveJoystickVertical = new Joystick(RobotMap.DRIVE_RIGHT_JOYSTICK);
		operator = new Operator();
		driver = new Driver();
	}
	
    public class Operator {

		public boolean intakeUp() {
			return operatorJoystick.getRawButton(10);
		}
		
		public boolean intakeDeploy(){
			return operatorJoystick.getRawButton(9);
		}
		
		public boolean runIntake() {
			return operatorJoystick.getRawButton(8);
		}
    	
		public boolean purgeIntake() {
			return operatorJoystick.getRawButton(7);
		}
		
//		public double shooterPower() {
//			return operatorJoystick.getY();
//		}
		public boolean shoot(){
			return operatorJoystick.getRawButton(1);
		}
//		public boolean loadBallFlywheels()
//		{
//			return operatorJoystick.getRawButton(5);
//		}
		public boolean shootBallFlywheels()
		{
			return operatorJoystick.getRawButton(2);
		}
		public boolean pivotCloseShot()
		{
			return operatorJoystick.getRawButton(6);
		}
		public boolean pivotFarShot()
		{
			return operatorJoystick.getRawButton(4);
		}
		public boolean pivotReset(){
			return  operatorJoystick.getRawButton(3); //operatorJoystick.getRawButton(5) &&
		}
		public boolean pivotStoreState()
		{
			return operatorJoystick.getRawButton(5);
		}

		public boolean isSemiAutonomousIntakePressed()
		{
			return operatorJoystick.getRawButton(12);
		}
		
		public boolean isDefenseState(){
			return operatorJoystick.getRawButton(11);
		}
		
		
		//***MANUALS***//
		public boolean isManualFirePiston(){
			return operatorJoystick.getPOV() == 180;
		}
		
		public boolean isManualPivotReset()
		{
			return operatorJoystick.getRawButton(8);
		}
		public double getManualPower()
		{
			return operatorJoystick.getY();
		}
		public boolean isManualOverrideOperator(){
			return operatorJoystick.getPOV() == 0;
		}
    }
    
    public class Driver {

		public double getForward() {
			// TODO Auto-generated method stub
			return driveJoystickVertical.getY();
		}
    	
		public double getRotation() {
			return -driveJoystickHorizontal.getX();
		}
		
		public boolean isDrivetrainHighGearButtonPressed() {
            return driveJoystickHorizontal.getRawButton(1) || driveJoystickVertical.getRawButton(1);
        }
		
		public boolean isCameraUpPressed(){
			return driveJoystickHorizontal.getRawButton(11) || driveJoystickVertical.getRawButton(6);
		}
		
		public boolean isManualOverride(){
			return driveJoystickHorizontal.getRawButton(2) || driveJoystickVertical.getRawButton(2)
					|| operator.isManualOverrideOperator();
		}
		
		public boolean isDefenseShot(){
			return driveJoystickHorizontal.getRawButton(9) || driveJoystickVertical.getRawButton(9);
		}
    }
    
    public class Manual {
    	
    }

	public boolean autoAim() {
		// TODO Auto-generated method stub
		return driveJoystickHorizontal.getRawButton(3) || driveJoystickVertical.getRawButton(3);
	}
}

