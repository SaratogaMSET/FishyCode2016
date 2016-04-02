package org.usfirst.frc.team649.robot.commands;

import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class VisionLoop extends Command {
	public Timer timer, secondaryTimer;
	public double nextRunTime;
	public Mat image;
	
	public static double TIME_INTERVAL = 0.25;
	
	public VisionLoop(){
		timer = new Timer();
		secondaryTimer = new Timer(); 
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
		nextRunTime = 0 + TIME_INTERVAL;
	}
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		timer.start();
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		if (Robot.runVision){
			if (timer.get() > nextRunTime){
				nextRunTime +=TIME_INTERVAL;
				
				image = new Mat();
				
				double _time = timer.get();
				if (Robot.camera.noOpencvErrors){
					System.out.println("PROCESSING IMAGE AT: " + timer.get() + ", Still no OPENCV errors!");
					Robot.camera.vcap.read(image);
					
					
					Robot.currCenter = Robot.camera.findOneRetroTarget(image);
					double delta_t = timer.get() - _time;
					System.out.println("CV Processing Time: " + delta_t);
				}
				
				
			}
		}
		else{
			timer.reset();
			timer.start();
			nextRunTime = 0 + TIME_INTERVAL;
		}
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		Robot.camera.vcap.release();
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub

	}

}
