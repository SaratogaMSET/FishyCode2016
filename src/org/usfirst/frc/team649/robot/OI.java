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

		public boolean toggleIntake() {
			// TODO Auto-generated method stub
			return operatorJoystick.getRawButton(1);
		}
		
		public boolean runIntake() {
			return operatorJoystick.getRawButton(12);
		}
    	
		public boolean purgeIntake() {
			return operatorJoystick.getRawButton(11);
		}
		
		public double shooterPower() {
			return operatorJoystick.getY();
		}
		public boolean shoot(){
			return operatorJoystick.getRawButton(8);
		}
		public boolean loadBallFlywheels()
		{
			return operatorJoystick.getRawButton(4);
		}
		public boolean shootBallFlywheels()
		{
			return operatorJoystick.getRawButton(6);
		}
		public boolean pivotShootState()
		{
			return operatorJoystick.getRawButton(5);
		}
		public boolean resetPivot()
		{
			return operatorJoystick.getRawButton(3);
		}
		public boolean pivotStoreState()
		{
			return operatorJoystick.getRawButton(9);
		}
		public boolean isManualPivotRest()
		{
			return operatorJoystick.getRawButton(2);
		}
		public boolean isSemiAutonomousIntakePressed()
		{
			return operatorJoystick.getRawButton(10);
		}
		public double getManualPower()
		{
			return operatorJoystick.getY();
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
		
		public boolean isDrivetrainLowGearButtonPressed() {
            return driveJoystickHorizontal.getRawButton(1) || driveJoystickVertical.getRawButton(1);
        }
		
		public boolean isCameraUpPressed(){
			return driveJoystickHorizontal.getRawButton(10) || driveJoystickVertical.getRawButton(7);
		}
		
		public boolean isManualOverride(){
			return driveJoystickHorizontal.getRawButton(2) || driveJoystickVertical.getRawButton(2);
		}
    }
    
    public class Manual {
    	
    }
}

