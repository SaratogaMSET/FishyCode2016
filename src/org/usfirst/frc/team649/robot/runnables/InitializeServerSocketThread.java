package org.usfirst.frc.team649.robot.runnables;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;

import org.usfirst.frc.team649.robot.Robot;

public class InitializeServerSocketThread implements Runnable {
	
	public static int PORT = 5050;
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		try{
			ServerSocket serverSocket = new ServerSocket();
			serverSocket.setReuseAddress(true);
			serverSocket.bind(new InetSocketAddress(PORT));

			System.out.println("Server is running and listening...");
			
			while (Robot.robotEnabled){
				Socket socket = serverSocket.accept();
				DataInputStream dis = new DataInputStream(socket.getInputStream());
				System.out.println("Recieved from Client: " + dis.readUTF());
				dis.close();
				socket.close();
			}
			
			serverSocket.close();

		}
		catch(Exception e){
			e.printStackTrace();
			
		}

	}

}
