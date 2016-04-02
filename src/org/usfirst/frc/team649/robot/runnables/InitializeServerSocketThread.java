package org.usfirst.frc.team649.robot.runnables;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;

import org.usfirst.frc.team649.robot.Robot;
import org.usfirst.frc.team649.robot.util.Center;

import edu.wpi.first.wpilibj.Timer;

//stands on its own, run at the beginning of match, runs in parallel and does all the work
public class InitializeServerSocketThread implements Runnable {
	
	public Timer t;
	
	@Override
	public void run() {
		// TODO Auto-generated method stub

		t.reset();
		t.start();
		
		try{
			ServerSocket serverSocket = new ServerSocket();
			serverSocket.setReuseAddress(true);
			serverSocket.bind(new InetSocketAddress(Robot.PORT));
			
			System.out.println("Server is running and listening...Boot Up Time: " + t.get());
			Robot.isRIOServerStarted = true; //must be called before starting RateChecker
			
			RateChecker rateChecker = new RateChecker();
			//this just updates the isReceivingData boolean to reflect whether or not we are getting a constant stream of data
			rateChecker.start(); //we relinquish control of the timer here bc its used in RateChecker
			
			
			while (Robot.robotEnabled){
				
				Socket socket = serverSocket.accept();
				rateChecker.alertPackageReceived();
				
				//READ ONCE MESSAGE RECIEVED
				DataInputStream dis = new DataInputStream(socket.getInputStream());
				String s = dis.readUTF();
				String[] message = s.split(",");
				//UPDATE CENTER
				Robot.currCenter = new Center(Double.parseDouble(message[0]), Double.parseDouble(message[1]));
//				System.out.println("Received from Client: " + dis.readUTF());
				dis.close();
				socket.close();
			}
			
			serverSocket.close();
			Robot.isRIOServerStarted = false;

		}
		catch(Exception e){
			e.printStackTrace();
			
		}

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
					return; //end thread if its taking too long for server to boot
				}
			}
			
			t.reset();
			t.start();
			
			while (Robot.isRIOServerStarted){
				if (t.get() - timeSinceLastAlert > Robot.MAX_PERIOD_BETWEEN_RECIEVING_DATA){
					Robot.isReceivingData = false;
				}
				else{
					Robot.isReceivingData = true;
				}
			}
		}
		
		public void alertPackageReceived(){
			timeSinceLastAlert = t.get();
		}
		
	}

}
