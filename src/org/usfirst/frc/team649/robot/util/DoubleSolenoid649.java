package org.usfirst.frc.team649.robot.util;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleSolenoid649 {
	Solenoid sol1;
	Solenoid sol2;
	
	public DoubleSolenoid649(int m1, int p1, int m2, int p2){
		sol1 = new Solenoid(m1, p1);
		SmartDashboard.putString("Sol1", "" + m1 + " " + p1);
		sol2 = new Solenoid(m2, p2);
		SmartDashboard.putString("Sol2", "" + m2 + " " + p2);
	}
	
	public void set(boolean curState){
		sol1.set(curState);
		sol2.set(!curState);
	}
	public boolean get(){
		return sol1.get();
	}
}
