package org.usfirst.frc.team649.robot.runnables;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Calendar;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.util.Center;

public class PullVisionTxtThread implements Runnable {
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		try {
			long _time = Calendar.getInstance().getTimeInMillis();
			//System.out.println("Starting TXT pull @ " + _time);
			Process p = Runtime.getRuntime().exec(Robot.pullPath);
			try {
				int exit = p.waitFor();
				updateCenterFromFile();
				
				//System.out.println("Pulled TXT, TIME TAKEN: " + (Calendar.getInstance().getTimeInMillis() - _time));
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	public void updateCenterFromFile(){
		try {
            File filepath = new File(Robot.visionFile);  // file path to save
            BufferedReader in = new BufferedReader(new FileReader(filepath));

            String str1 = in.readLine();
//            System.out.println("FIRST LINE: " + str1);
            String[] line1 = str1.split(",");
            
            if (line1.length >= 2){
            	Robot.currCenter = new Center(Double.parseDouble(line1[0]), Double.parseDouble(line1[1]));
            }
        }
        catch (Exception e){
            System.out.println("Exception: File READ failed: " + e.toString());
            //default
            Robot.currCenter = new Center(-1,-1);
        }
	}

}
