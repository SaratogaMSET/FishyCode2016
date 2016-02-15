package org.usfirst.frc.team649.robot.util;

import edu.wpi.first.wpilibj.Solenoid;

public class DoubleSolenoid649 {
	static Solenoid sol1;
	static Solenoid sol2;
	
	public DoubleSolenoid649(int m1, int p1, int m2, int p2){
		sol1 = new Solenoid(m1, p1);
		sol2 = new Solenoid(m2, p2);
	}
	
	public void set(boolean curState){
		sol1.set(curState);
		sol2.set(!curState);
	}
	public boolean get(){
		return sol1.get();
	}
}
