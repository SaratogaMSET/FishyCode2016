package org.usfirst.frc.team649.robot.util;

public class DoubleSolenoid {
	static Solenoid sol1;
	static Solenoid sol 2
	public DoubleSolenoid(int m1, int p1, int m2, int p2){
		sol1 = new Solenoid();
		so12 = new Solenoid();
	}
	public static void setForward(){
		sol1.set(true);
		sol2.set(false);
	}
	public static void setBack(){
		sol1.set(false);
		sol2.set(true);
	}
	public static boolean get(){
		return sol1.get()
	}
}
