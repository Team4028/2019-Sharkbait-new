package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;

// this class encapsulates interactions with the NavX Sensor
// you must setup path to libraries using these instructions:
// http://www.pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
// http://www.pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/
//=====> For Changes see Seabass
public class NavXGyro {
	public static NavXGyro _instance = new NavXGyro();
	
	public static NavXGyro getInstance() {
		return _instance;
	}
	
	private AHRS _navXSensor;
	
	public NavXGyro() {
        try {          
        	_navXSensor = new AHRS(RobotMap.NAVX_PORT); // Communication via RoboRIO MXP (SPI) 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
	}
	//============================================================================================
	// Methods follow
	//============================================================================================
	public double getYaw() { return _navXSensor.getYaw(); }
	
	public void zeroYaw()  { _navXSensor.zeroYaw(); }
	
	// update the Dashboard with any NavX specific data values
	public void OutputToSmartDashboard() {}
	
	public void UpdateLogData(LogData logData) {}
}