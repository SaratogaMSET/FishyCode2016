package org.usfirst.frc.team649.robot.runnables;

import java.io.IOException;
import java.util.Calendar;

import org.usfirst.frc.team649.robot.Robot;

public class PullVisionTxtThread implements Runnable {
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		try {
			System.out.println("About to pull TXT @ " + Calendar.getInstance().getTime());
			Process p = Runtime.getRuntime().exec(Robot.pullPath);
			try {
				int exit = p.waitFor();
				System.out.println("Pulled TXT @ " + Calendar.getInstance().getTime());
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
