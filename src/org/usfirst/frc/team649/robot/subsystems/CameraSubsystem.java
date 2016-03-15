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
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.usfirst.frc.team649.robot.RobotMap;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class CameraSubsystem extends Subsystem {
	public VideoCapture vcap;
	public Servo camServo;
	
	public boolean noOpencvErrors;
	
	public static double CAM_UP_ANGLE = 5; //TODO
	public static double CAM_DOWN_ANGLE = 110;
	
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
	
	public static String image_1 = "/home/lvuser/frc_images/from\\ position\\ 1.jpg";
	public static String image_2 = "/home/lvuser/frc_images/from\\ position\\ 2.jpg";
	public static String image_3 = "/home/lvuser/frc_images/from\\ position\\ 3.jpg";
	public static String image_4 = "/home/lvuser/frc_images/from\\ position\\ 4.jpg";
	public static String image_5 = "/home/lvuser/frc_images/from\\ position\\ 5.jpg";
	
	Mat image, imageHSV, erode, dilate, hierarchy;
	List<MatOfPoint> contours;
	
	
	public CameraSubsystem(String ip){
		camServo = new Servo(RobotMap.Drivetrain.CAM_SERVO);
	
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
		
		
		try{
			
    		//FOR Axis CAMERA 206
    		//vcap = new VideoCapture("http://root:admin@axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
    		
    		//FOR Axis M1011
    		vcap = new VideoCapture("http://axis-camera.local/axis-cgi/mjpg/video.cgi?user=root&password=admin&channel=0&.mjpg");
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
	
	public void setServoAngle(double angle){
		camServo.setAngle(angle);
	}
	
	 public Center findOneRetroTarget(Mat image){
	    	
	    	Center center =  new Center(-1,-1); //default
	    	//image = new Mat();
	    	imageHSV = new Mat();
	    	
	    	Imgproc.cvtColor(image, imageHSV, Imgproc.COLOR_BGR2HSV);
	    	
	    	//Core.inRange(imageHSV, new Scalar(78, 124, 213), new Scalar(104, 255, 255), imageHSV);

	    	Core.inRange(imageHSV, new Scalar(60, 41, 218), new Scalar(94, 255, 255), imageHSV);
	    	
	    	//BLUR
	    	Imgproc.GaussianBlur(imageHSV, imageHSV, new Size(5,5), 0);
	    	
	    	//DILATE > ERODE > DILATE 
	    	dilate = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));
	    	Imgproc.dilate(imageHSV, imageHSV, dilate);//dilate  
	    	erode = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
	    	Imgproc.erode(imageHSV, imageHSV, erode);
	    	Imgproc.dilate(imageHSV, imageHSV, dilate);
	    	
	    	//BLUR PART 2
	    	Imgproc.GaussianBlur(imageHSV, imageHSV, new Size(15,15), 0);
	    	
	    	//THRESHING
	    	Imgproc.threshold(imageHSV, imageHSV, 73, 255, Imgproc.THRESH_BINARY);
	    	
	    	//DILATE ONCE MORE
	    	Imgproc.dilate(imageHSV, imageHSV, dilate);
	    	
	    	/************/
	    	//CONTOURS AND OBJECT DETECTION
	    	contours = new ArrayList<>();
	    	hierarchy = new Mat();
	    	
	    	Moments mu;
	    	
	    	// find contours
	    	Imgproc.findContours(imageHSV, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
	    	
	    	// if any contour exist...
	    	if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
	    	{
	    		int largest = 0;
	    		
		        // for each remaining contour, find the biggest
		        for (int i = 0; i < contours.size(); i++)
		        {
		        	double area = Imgproc.contourArea(contours.get(i));
		        	mu = Imgproc.moments(contours.get(i));
		        	double y_coord = mu.get_m01()/mu.get_m00();
		        	//greater than min size AND in the upper part of photo AND greater than the last biggest
		        	if (area > 100.0  &&  y_coord < MAX_Y_COORD && area > Imgproc.contourArea(contours.get(largest))){
		        		
		        		largest = i;
			        	//NetworkTable tab = NetworkTable.getTable("Obj " + i);
		        		
		        		//Center:    mu.m10()/mu.m00() , mu.m01()/mu.m00()
			        	
		  	        }
		        }
		        
		        //print details of the biggest one
	        	String name = "Obj " + largest;
	        	Rect r = Imgproc.boundingRect(contours.get(largest));
	        	mu = Imgproc.moments(contours.get(largest));
	        	
	        	//"("+ mu.get_m10()/mu.get_m00() + ", " + mu.get_m01()/mu.get_m00() + ")");
	        	
	        	//ASSUME LARGEST is the target, now calc dist
	         	
	        	double dist = calcDistAxis206(r.width, WIDTH_TARGET, 320, STANDARD_VIEW_ANGLE);
	        	center = new Center(mu.get_m10()/mu.get_m00(), mu.get_m01()/mu.get_m00());
	        	
	        	String[] keys = new String[]{"Obj Center X: ", "Obj Center Y: ", "Obj Area: ", "Obj width: ", "Obj height: ", "Obj Distance: "}; 
	        	Object[] elements = new Object[]{(Double)center.x, (Double)center.y, (Double)Imgproc.contourArea(contours.get(largest)), (Integer)r.width, (Integer)r.height, (Double)dist};
//	        	"Obj 0 Center X: ", mu.get_m10()/mu.get_m00());
//	        	"Obj 0 Center Y: ", mu.get_m01()/mu.get_m00());
//	        	"Obj 0 Area: ", Imgproc.contourArea(contours.get(largest)));
//	        	"Obj 0 width: ", r.width);
//	        	"Obj 0 height: ", r.height);
//	        	"Obj 0 Distance: ", dist);	
//	        	SendableTable contourObj = new SendableTable(name, keys, elements);
//	        	SmartDashboard.putData("Obj " + largest, contourObj);

	        	
//	        	SmartDashboard.putNumber("Obj 0 Center X: ", center.x);
//	        	SmartDashboard.putNumber("Obj 0 Center Y: ", center.y);
//	        	SmartDashboard.putNumber("Obj 0 Area: ", Imgproc.contourArea(contours.get(largest)));
//	        	SmartDashboard.putNumber("Obj 0 width: ", r.width);
//	        	SmartDashboard.putNumber("Obj 0 height: ", r.height);
//	        	SmartDashboard.putNumber("Obj 0 Distance: ", dist);
	        	
	        	//Imgcodecs.imwrite("output.png", imageHSV);
	    	}
	    	else{
//	    		SmartDashboard.putNumber("Obj 0 Center X: ", 0);
//	        	SmartDashboard.putNumber("Obj 0 Center Y: ", 0);
//	        	SmartDashboard.putNumber("Obj 0 Area: ", 0);
//	        	SmartDashboard.putNumber("Obj 0 width: ", 0);
//	        	SmartDashboard.putNumber("Obj 0 height: ", 0);
//	        	SmartDashboard.putNumber("Obj 0 Distance: ", 0);
	    	}
	    	
	    	
//	    	SmartDashboard.putNumber("Number of contours in image", contours.size());
//	    	SmartDashboard.putNumber("Mat Height", image.height());
//	    	SmartDashboard.putNumber("Mat Width", image.width());
	    	
	        //mem save
	        image.release();
	    	imageHSV.release();
	    	erode.release();
	    	dilate.release();
	    	hierarchy.release();
	        System.gc();
	        
	        return center;
    }
	 
	 public double calcDistAxis206(double obj_pix, double obj_in, double view_pix, double max_cam_angle){
    	return view_pix * obj_in / (2*Math.tan(max_cam_angle) * obj_pix);
    }
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
