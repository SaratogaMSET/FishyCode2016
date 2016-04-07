package org.usfirst.frc.team649.robot.subsystems;

//import java.awt.Graphics;
//import java.awt.Graphics2D;
//import java.awt.Image;
//import java.awt.Toolkit;
//import java.awt.image.BufferedImage;
//import java.awt.image.DataBufferByte;
//import java.io.IOException;
//import java.io.InputStream;
//import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.RobotMap.Drivetrain;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class CameraSubsystem extends Subsystem {
	public VideoCapture vcap;
	public Servo camServo;
	public DoubleSolenoid camPiston;
	
	public boolean noOpencvErrors;
	
	public static boolean CAM_UP = true;
	
	public static double WIDTH_TARGET = 18.5; //in
	public static double STANDARD_VIEW_ANGLE = 0.454885;//0.9424778; //radians, for an Axis Camera 206 /////...54 degrees
	public static double MAX_Y_COORD = 195; //TODO find the actual angle of camera and the corresponding max y coord	
	public static double X_TARGET = 160;
	public static double K_PIX = 1.0/400;
	
	public static double POS_1_CAM_X = 160; //pixels
	public static double POS_2_CAM_X = 160; //pixels
	public static double POS_3_CAM_X = 160; //pixels
	public static double POS_4_CAM_X = 160; //pixels
	public static double POS_5_CAM_X = 160; //pixels
	
	
	public CameraSubsystem(String ip){
		//camServo = new Servo(RobotMap.Drivetrain.CAM_SERVO);
		camPiston = new DoubleSolenoid(Drivetrain.SOLENOID_PORTS[0], Drivetrain.SOLENOID_PORTS[1], Drivetrain.SOLENOID_PORTS[2]);
		
		//System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
		
		
		try{
			
    		//FOR Axis CAMERA 206
    		//vcap = new VideoCapture("http://root:admin@axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
    		
    		//FOR Axis M1011
    //		vcap = new VideoCapture("http://axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
    		//vcap.open("http://169.254.110.201//mjpg/video.mjpg?user=root&password=admin&channel=0&.mjpg");
    		
    		//while (!vcap.isOpened()){}
    		//System.out.println("R-INIT FINISHED VCAP START AND FOUND CAM");
    		//FOR USB CAMERA
//    		vcap = new VideoCapture(0);
//    		//Thread.sleep(1000);
    		noOpencvErrors = true;
		}
    	catch (Exception e){
    		System.out.println("\nERROROROROROROROR WITH CAM");
    		System.out.println(e.getMessage() + "\n");
    		noOpencvErrors = false;
    	}
	}

	 public void setCamera(boolean up){
		 camPiston.set(up ? Value.kForward : Value.kReverse);
	 }
	 
	 public double calcDistAndroidCam(double obj_pix, double obj_in, double view_pix, double max_cam_angle){
    	return view_pix * obj_in / (2*Math.tan(max_cam_angle) * obj_pix);
    }
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
