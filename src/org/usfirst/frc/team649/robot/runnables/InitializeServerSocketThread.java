package org.usfirst.frc.team649.robot.runnables;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.BindException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.Timer;

//stands on its own, run at the beginning of match, runs in parallel and does all the work
public class InitializeServerSocketThread extends Thread {
	
	public Timer t;
	public ServerSocket serverSocket;
	
	@Override
	
	public void run() {
		// TODO Auto-generated method stub

		t = new Timer();
		
		t.reset();
		t.start();
		
		try {
			Process p = Runtime.getRuntime().exec(Robot.initPath);
//			p.waitFor();
			System.out.println("ADB SET UP DONE");
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
			Robot.logMessage("ADB SET UP FAILED");
		} 
//		catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
		try{
			serverSocket = new ServerSocket();
			serverSocket.setReuseAddress(true);
			try {
				serverSocket.bind(new InetSocketAddress(Robot.PORT));
			} catch (BindException e){
				System.out.println("Caught a bind exception: SERVER ALREADY UP"); // <--- that is very bad and will cause quits
				//server
				Robot.logMessage("Caught a bind exception: SERVER ALREADY UP");
			}
			
			System.out.println("Server is running and listening...Boot Up Time: " + t.get());
			Robot.logMessage("Server is running and listening...Boot Up Time: " + t.get());
			
			Robot.isRIOServerStarted = true; //must be called before starting RateChecker
			
			RateChecker rateChecker = new RateChecker();
			//this just updates the isReceivingData boolean to reflect whether or not we are getting a constant stream of data
			rateChecker.start(); //we relinquish control of the timer here bc its used in RateChecker
			
			
			while (Robot.robotEnabled){
				
				Socket socket = serverSocket.accept();
//				System.out.println("Recieved accept");
				rateChecker.alertPackageReceived();
				
				//READ ONCE MESSAGE RECIEVED
				DataInputStream dis = new DataInputStream(socket.getInputStream());
				String s = dis.readUTF();
				//System.out.println("Recieved: " + s);
				String[] message = s.split(",");
				//UPDATE CENTER
				Robot.updateCenter(  new Center(Double.parseDouble( message[0]), Double.parseDouble(message[1]) )  );
//				Robot.droidIP = message[2];
//				System.out.println("Received from Client: " + dis.readUTF());
				dis.close();
				socket.close();
				
			}
			
			System.out.println("Socket about to be closed");
			serverSocket.close();
			Robot.logMessage("Socket Closed");
			Robot.isRIOServerStarted = false;

		}
		catch(SocketException e){
			System.out.println("Server Socket ---> " + e.getMessage());
		}
		catch(Exception e){
			e.printStackTrace();
			
		}

		System.out.println("VISION THREAD ENDING");
	}
	
	@Override
	public void interrupt(){
//		System.out.println("CALLED interrupt");
		if (serverSocket != null){
			try {
				serverSocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				System.out.println("Failed closing on interrupt: " + e.getMessage());
			}
		}
		super.interrupt();
	}
	
	//just updates variable, doesnt really do anything else
	public class RateChecker extends Thread{

		double timeSinceLastAlert;
		
		public RateChecker(){
			timeSinceLastAlert = 0; //time since update
		}
		
		@Override
		public void run() {
			t.reset(); //reset
			t.start();
			
			while (!Robot.isRIOServerStarted){
				if (t.get() > 15){
					System.out.println("RATE CHECKER: CONNECTION TIMED OUT");
					Robot.logMessage("Initialize socket: CHECKER > CONNECTION TIMED OUT");
					return; //end thread if its taking too long for server to boot
				}
			}
			
			t.reset();
			t.start();
			
			while (Robot.isRIOServerStarted){
				if (t.get() - timeSinceLastAlert > Robot.MAX_PERIOD_BETWEEN_RECIEVING_DATA){
					Robot.isReceivingData = false;
					Robot.rateCenterUpdated = 0;
				}
				else{
					Robot.isReceivingData = true;
					
					double diff = Robot.currCenter.x - Robot.GOOD_X; //positive means turn right
					
					if(Robot.currCenter.x == -1) {
						Robot.drivetrain.setLEDs(DrivetrainSubsystem.RED, DrivetrainSubsystem.RED);
					} else if (diff < -8) {
						Robot.drivetrain.setLEDs(DrivetrainSubsystem.RED, DrivetrainSubsystem.GREEN);
					} else if(diff > 8) {
						Robot.drivetrain.setLEDs(DrivetrainSubsystem.GREEN, DrivetrainSubsystem.RED);
					} else if(Math.abs(diff) < 8) {
						Robot.drivetrain.setLEDs(DrivetrainSubsystem.GREEN, DrivetrainSubsystem.GREEN);
					}
					
				}
			}
		}
		
		public void alertPackageReceived(){
			//System.out.println("ALERTED THREAD");
			timeSinceLastAlert = t.get();
		}
		
	}
	

}
