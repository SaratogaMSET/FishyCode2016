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
		operator = new Operator();
	}
    public class Operator {

		public boolean isIntakeDeploy() {
			// TODO Auto-generated method stub
			return false;
		}
		
		public boolean isIntakeRetract() {
			return false;
		}
    	
    }
    
    public class Driver {
    	
    }
    
    public class Manual {
    	
    }
}

