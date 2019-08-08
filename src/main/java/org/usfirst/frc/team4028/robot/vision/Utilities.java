package org.usfirst.frc.team4028.robot.vision;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.DriverStation;

public class Utilities 
{
	// define class level constants
	private static final int PING_TIMEOUT_MSEC = 200;
	
	// simple test is to check if we can get a ping response from the server
	// 	this will tell us if the remote server is accessible (but not necessarily if RoboRealm is up)
	public static boolean SimplePingTest(String hostNameOrIPv4Addr)
	{
		boolean isPingable = false;
		
    	try 
    	{
			InetAddress inet = InetAddress.getByName(hostNameOrIPv4Addr);
		
			// try to ping the remote server
			if(inet.isReachable(PING_TIMEOUT_MSEC))
	        {
				DriverStation.reportWarning("CAN Ping Kangaroo [" + hostNameOrIPv4Addr + "]", false);
				isPingable = true;
	        }
	        else
	        {
	        	DriverStation.reportError("CANNOT Ping Kangaroo [" + hostNameOrIPv4Addr + "]", false);
	        }
		} 
    	catch (UnknownHostException e) 
    	{
    		DriverStation.reportError("CANNOT Ping Kangaroo [" + hostNameOrIPv4Addr + "]", true);

		} 
    	catch (IOException e) 
    	{
			DriverStation.reportError("CANNOT Ping Kangaroo [" + hostNameOrIPv4Addr + "]", true);
		}
    	
    	return isPingable;
	}	
	
	// more robust test to check if the RoboRealm Port on the remote server is active
	//	this really implies that RoboRealm is running
	public static boolean RobustPortTest(String hostNameOrIPv4Addr, int portNo)
	{
        Socket socket = null;
        String message = null;
        boolean isRomotePortAvailable = false;	// default to false
        
        try 
        {
            socket = new Socket();
            socket.setReuseAddress(true);
            
            // try to connect to the host & port using a Socket
            SocketAddress socketAddr = new InetSocketAddress(hostNameOrIPv4Addr, portNo);
            socket.connect(socketAddr, PING_TIMEOUT_MSEC);
        } 
        catch (IOException e) 
        {
            if ( e.getMessage().equals("Connection refused")) 
            {
                message = "Port: [" + portNo + "] on host: [" + hostNameOrIPv4Addr + "] is closed.";
            }
            
            if ( e instanceof UnknownHostException ) 
            {
                message = "Node: [" + hostNameOrIPv4Addr + "] cannot be resolved.";
            }
            
            if ( e instanceof SocketTimeoutException ) 
            {
                message = "Timeout while attempting to reach node: [" 
                				+ hostNameOrIPv4Addr 
                				+ "] on port: [" 
                				+ portNo + "]";
            }
        } 
        finally 
        {
            if (socket != null) 
            {
                if ( socket.isConnected()) 
                {
                	message = ("VISION TEST SUCCESS, Port: [" + portNo + "] on Remote Node: [" + hostNameOrIPv4Addr + "] IS reachable!");
                	
                	// this is the single success case
                    isRomotePortAvailable = true;
                } 
                else 
                {
                	message = "VISION TEST FAILURE, Port: [" + portNo + "] on remote node: [" + hostNameOrIPv4Addr 
                				+ "] IS NOT reachable [ " + message + "]";
                }
                
                // clean up
                try 
                {
                    socket.close();
                } 
                catch (IOException e) 
                {
                }
            }
        }

        // write success or failure msg to the drivers station
        if(isRomotePortAvailable)
        {
        	// write success to drivers station
        	DriverStation.reportWarning(message, false);
        }
        else
        {
        	// write failure to drivers station
        	DriverStation.reportError(message, false);
        }
        
		return isRomotePortAvailable;
	}
}