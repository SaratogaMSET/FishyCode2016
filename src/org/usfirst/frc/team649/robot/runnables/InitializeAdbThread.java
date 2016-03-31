package org.usfirst.frc.team649.robot.runnables;

import java.io.IOException;

import org.usfirst.frc.team649.robot.Robot;

public class InitializeAdbThread implements Runnable {
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		try {
			Process p = Runtime.getRuntime().exec(Robot.initPath);
			try {
				int exit = p.waitFor();
				System.out.println("Finished Initialization");
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
