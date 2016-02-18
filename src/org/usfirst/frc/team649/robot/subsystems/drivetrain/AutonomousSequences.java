package org.usfirst.frc.team649.robot.subsystems.drivetrain;

public class AutonomousSequences {
	
	/*
	* pos = defense location
	* originalDiff = the original difference when vision offset was calculated
	* x = current index of array used to calculate motor offset
	* 
	* returns the motor power offsets {left, right}
	*/
	public static double[] visionOffset(int pos, double originalDiff, int x){
		double[] error = {0.0, 0.0}; 
		
		//TODO fill in each function
		switch (pos){
			case 1:  			///LOW BAR
				
				break;
			case 2:				///POS 2
				
				break;
			case 3:				///POS 3
				
				break;
			case 4:				///POS 4
				
				break;
			case 5:				///POS 5
				
				break;
			default:
				error = new double[]{0.0, 0.0};
				break;	
		}
		
		return error;
	}
	
	
	
	public static double[][] fromPos1 = {
			{0,0,0,0,0},
			{0,0,0,0,0}
	};
	public static double[][] fromPos2 = {
			{0,0,0,0,0},
			{0,0,0,0,0}
	};
	public static double[][] fromPos3 = {
			{0,0,0,0,0},
			{0,0,0,0,0}
	};
	public static double[][] fromPos4 = {
			{0,0,0,0,0},
			{0,0,0,0,0}
	};
	public static double[][] fromPos5 = {
			{0,0,0,0,0},
			{0,0,0,0,0}
	};
	
}
