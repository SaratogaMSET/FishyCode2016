package org.usfirst.frc.team649.robot.runnables;

import java.io.IOException;

import org.usfirst.frc.team649.robot.Robot;

public class EndAppThread implements Runnable {
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		try {
			Process p = Runtime.getRuntime().exec(Robot.endPath);
			try {
				int exit = p.waitFor();
				System.out.println("App Closed");
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
