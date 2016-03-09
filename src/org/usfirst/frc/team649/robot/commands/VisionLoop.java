package org.usfirst.frc.team649.robot.commands;

import org.opencv.core.Mat;
import org.usfirst.frc.team649.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class VisionLoop extends Command {
	Timer timer;
	double nextRunTime;
	Mat image;
	
	public static double TIME_INTERVAL = 0.25;
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
		timer = new Timer();
		timer.start();
		nextRunTime = TIME_INTERVAL;
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		if (timer.get() > nextRunTime){
			nextRunTime +=TIME_INTERVAL;
			
			image = new Mat();
			
			if (Robot.camera.noOpencvErrors)
				System.out.println("Still no OPENCV errors");
				Robot.camera.vcap.read(image);
				
				Robot.currCenter = Robot.camera.findOneRetroTarget(image);
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
