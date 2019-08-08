package org.usfirst.frc.team4028.robot.sensors;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Date;
import java.util.TimerTask;
import java.util.Vector;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.opencv.core.Mat;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.VISION_CAMERAS;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.vision.Dimension;
import org.usfirst.frc.team4028.robot.vision.RoboRealmAPI;
import org.usfirst.frc.team4028.robot.vision.Utilities;
import org.usfirst.frc.team4028.robot.vision.RawImageData;
import org.usfirst.frc.team4028.robot.subsystems.DashboardInputs;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is a wrapper around the interface to RoboRealm over TCP Sockets hosted on a 
 * on-robot Kangaroo
 * 
 *     	//_roboRealmClient.ChangeToCamera(ViSION_CAMERAS.BOILER);
 */
public class RoboRealmClient {
	public static RoboRealmClient _instance = new RoboRealmClient(RobotMap.KANGAROO_IPV4_ADDR, RobotMap.RR_API_PORT, RobotMap.LED_RINGS_DIO_PORT);
	
	public static RoboRealmClient getInstance() {
		return _instance;
	}
	
	// Define local working variables
 	private RoboRealmAPI _rrAPI;
 	
	//private java.util.Timer _updaterTimer; 
 	//private RoboRealmUpdater _task; 

 	private boolean _isConnected;
 	private String _currentVisionCameraName;
 	
 	private static final int TARGET_MINIMUM_Y_VALUE = 413;
 	
 	private static final int SOUTHWEST_X_IDX = 0;
 	private static final int SOUTHWEST_Y_IDX = 1;
 	private static final int SOUTHEAST_X_IDX = 2;
 	private static final int SOUTHEAST_Y_IDX = 3;
 	private static final int HIGH_MIDDLE_Y_IDX = 4;
 	private static final int  RAW_WIDTH_IDX = 5;
 	private static final int  RAW_HEIGHT_IDX = 6;
 	private static final int BLOB_COUNT_IDX = 7;
 	private static final int CAMERA_TYPE_IDX = 8;
 	
 	private static final int BAD_DATA_COUNTER_THRESHOLD = 10;
 	private static final int POLLING_CYCLE_IN_MSEC = 50; //100;
 	private static final int MAX_LOCK_WAIT_TIME_MSEC = 5;

 	private static final int EXPECTED_ARRAY_SIZE = 9;
 	private static final int EXPECTED_GEAR_BLOB_COUNT = 2;
 	private static final int EXPECTED_BOILER_BLOB_COUNT = 1;
 	
 	private static final double CAMERA_FOV_HORIZONTAL_DEGREES = 83.0; // 58.5;
 	private static final double CAMERA_TO_SHOOTER_OFFSET_IN_DEGREES = 0.7;
 	
 	// =============================================================
 	// Camera Adjustment Factor
 	// =============================================================
 	private static final double GEAR_CAMERA_CALIBRATION_FACTOR = 0.0;
 	private static final double BOILER_CAMERA_CALIBRATION_FACTOR = 0.0;
 	 	
 	private double _cameraCalibrationFactor = GEAR_CAMERA_CALIBRATION_FACTOR;
 	
 	private static final double BOILER_DISTANCE_OFFSET_INCHES = 2.0;	
 	
 	// =============================================================
 	
 	
 	// working variables raised to class level to reduce GC pressure inside update task
 	private Lock lock;
 	//private boolean _isVisionDataValid;
 	
 	private long _callElapsedTimeMSec;
 	private long _lastCallTimestamp;
 	private RawImageData _pollingThreadPublicTargetRawData;
 	private RawImageData _pollingThreadPrivateTargetRawData;
 	//private double _fovXCenterPoint;
 	//private double _targetXCenterPoint;
 	//private int _badDataCounter;
 	private long _lastDebugWriteTimeMSec;
 	//private final Object _targetDataMutex;
 	private int _expectedBlobCount;
 	//private double _degreesFromCenter;
 	//private Solenoid _gearCamLED;
 	//private Solenoid _shooterCamLED;
 	DigitalOutput _visionLedsRelay;
 	
	//============================================================================================
	// constructors follow
	//============================================================================================
    private RoboRealmClient(String kangarooIPv4Addr, int rrPortNo, int visioLEDsDIOPort)
    		//int PCMCanAddr, int gearCamLEDPCMPort, int shooterCamLEDPCMPort) 
    {        	
    	
    	// create an instance of the RoboRealm API client
    	_rrAPI = new RoboRealmAPI();
    	
    	//Utilities.SimplePingTest(kangarooIPv4Addr);
    	boolean isPingable = Utilities.RobustPortTest(kangarooIPv4Addr, rrPortNo);
    	
    	_isConnected = false;
    	// try to connect if pingable
    	if (isPingable) {
		    if (!_rrAPI.connect(kangarooIPv4Addr, rrPortNo)) {
		    	DriverStation.reportError("Could not connect to RoboRealm!", false);
		    	System.out.println("====> Could not connect to RoboRealm!");
		    	
		    	_isConnected = false;
		    } else {
		    	DriverStation.reportWarning("Connected to RoboRealm!", false);
		    	System.out.println("====> Connected to RoboRealm!");
		    	
		    	_isConnected = true;
		    }
    	}
        
		// create instance of the task the timer interrupt will execute
		//_task = new RoboRealmUpdater();
		
		// create a timer to fire events
		_lastCallTimestamp = new Date().getTime();
		//_updaterTimer = new java.util.Timer();
		//_updaterTimer.scheduleAtFixedRate(_task, 0, POLLING_CYCLE_IN_MSEC);
		
		if(_isConnected) {
			// start the camera thread
			Thread visionThread = new Thread(RoboRealmUpdater);
			visionThread.start();
		}
		
		//Initialize counter to indicate bad data until proved good
		//_badDataCounter = 10;
		
		// relay: https://wpilib.screenstepslive.com/s/4485/m/13809/l/599706-on-off-control-of-motors-and-other-mechanisms-relays
		//TODO: create the relay object
		//Set up LED PCM Rings
		//_gearCamLED = new Solenoid(PCMCanAddr, gearCamLEDPCMPort);
		//_shooterCamLED = new Solenoid(PCMCanAddr, shooterCamLEDPCMPort);
		//_visionLedsRelay = new DigitalOutput(visioLEDsDIOPort);
		//TurnAllVisionLEDsOff();
		
		if (_pollingThreadPrivateTargetRawData == null) {
 	 		_pollingThreadPrivateTargetRawData = new RawImageData();
 	 		_pollingThreadPrivateTargetRawData.FOVDimensions = new Dimension();
 	 	}
		
		// create lock object
		this.lock = new ReentrantLock();
    }
    
	//============================================================================================
	// Methods follow
	//============================================================================================
    public void TestConnect() {
    	Dimension fovDimensions = _rrAPI.getDimension();
    }
    
    public void TurnAllVisionLEDsOff()
    {
    	//_visionLedsRelay.set(false);
		//_gearCamLED.set(false);
		//_shooterCamLED.set(false);
    }
    
    public void TurnGearVisionLEDsOn()
    {
		//_shooterCamLED.set(false);
		//try {
		//	Thread.sleep(2);
		//} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
		//_gearCamLED.set(true);
    	//_visionLedsRelay.set(true);
    }
    
    public void TurnBoilerrVisionLEDsOn()
    {
    	//_gearCamLED.set(false);
		//try {
		//	Thread.sleep(2);
		//} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
		//_shooterCamLED.set(true);
    	//_visionLedsRelay.set(true);
    }
    
    // this method changes the active roborealm camera / program
    public void ChangeToCamera(VISION_CAMERAS visionCamera) {
    	_currentVisionCameraName = visionCamera.get_cameraName();
    	
    	switch(visionCamera) {
	    	case GEAR:
	        	_cameraCalibrationFactor = GEAR_CAMERA_CALIBRATION_FACTOR;
	        	_expectedBlobCount = EXPECTED_GEAR_BLOB_COUNT;
	        	//_gearCamLED.set(true);
	        	//_shooterCamLED.set(false);
	        	//_visionLedsRelay.set(true);
	        	
	    		break;
	    		
	    	case BOILER:
	        	_cameraCalibrationFactor = BOILER_CAMERA_CALIBRATION_FACTOR;
	        	_expectedBlobCount = EXPECTED_BOILER_BLOB_COUNT;
	        	//_gearCamLED.set(false);
	        	//_shooterCamLED.set(true);
	        	//_visionLedsRelay.set(true);
	    		break;
    	}
    	
    	if(_isConnected)
    	{
	    	if (_rrAPI.setVariable("CamType", _currentVisionCameraName)) {
	    		System.out.println("====> RoboRealm Camera switched to " + _currentVisionCameraName);
	    	}
	    	else {
	    		System.out.println("====> FAILED changing RoboRealm Camera to " + _currentVisionCameraName);
	    	}
    	}
 		else {
 			System.out.println("====> Warning..RoboRealm not connected!");
 		}
    }

    
 	// this method switches the currently running pipeline program
 	public void SwitchProgram(String pipeLineProgramFullPathName) {
 		if(_isConnected){
	 		Boolean isPauseOk = _rrAPI.pause();
	 		
	 		System.out.println("====> Prepare to Switch Program");
	 		
	 		if(isPauseOk) {
	        	//DriverStation.reportError("RoboRealm Program Paused successfully!", false);
	        	System.out.println("====> RoboRealm Program Paused succesfully!");
	        } else {
	        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
	        	System.out.println("====>Error..Could not pause RoboRealm program!");
	        }
	 		
	 		System.out.println("====> Chg Program To: " + pipeLineProgramFullPathName);
	 		
	 		Boolean isSwitchOk = _rrAPI.loadProgram(pipeLineProgramFullPathName);
	 		
	 		if(isSwitchOk) {
	        	//DriverStation.reportError("RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName, false);
	        	System.out.println("====> RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName);
	        } else {
	        	//DriverStation.reportError("Error..Could not switch RoboRealm program!", false);
	        	System.out.println("====> Error..Could not switch RoboRealm program!");
	        }	
	 		
	 		Boolean isResumeOk = _rrAPI.resume();
	 		
	 		if(isResumeOk) {
	        	//DriverStation.reportError("RoboRealm Program Paused succesfully!", false);
	        	System.out.println("====> RoboRealm Program Resumed succesfully!");
	        } else {
	        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
	        	System.out.println("====> Error..Could not resume RoboRealm program!");
	        }
 		}
 		else {
 			System.out.println("====> Warning..RoboRealm not connected!");
 		}
 	}
 	
	// update the Dashboard with any Vision specific data values
	public void OutputToSmartDashboard() {
		String dashboardMsg = "";
		
    	RawImageData publicTargetRawData = get_newTargetRawData();
		
		if( publicTargetRawData.IsVisionDataValid){
				dashboardMsg = "Camera= " + publicTargetRawData.CameraType
								+ " Angle= " + publicTargetRawData.FovCenterToTargetXAngleRawDegrees
								+ " MidHiY= " +  publicTargetRawData.HighMiddleY
								+ " mSec=" + publicTargetRawData.ResponseTimeMSec
								+ " Blob Count= " + publicTargetRawData.BlobCount 
								+ " Is on Gear Target= " + Boolean.toString(get_isInGearHangPosition());
				
		} else {
			dashboardMsg = "Vision DATA NOT VALID";
		}
		
		SmartDashboard.putString("VIsion", dashboardMsg);
		String distanceString = String.format("%.2f", publicTargetRawData.EstimatedDistance);
		SmartDashboard.putString("VIsion Distance", distanceString);
	} 	
	
	public void UpdateLogData(LogData logData) {
		RawImageData publicTargetRawData = get_newTargetRawData();
		
		if( publicTargetRawData.IsVisionDataValid){ 
			logData.AddData("RR:Camera", String.format("%s", _currentVisionCameraName.toString()));
			logData.AddData("RR:Angle", String.format("%.2f", publicTargetRawData.FovCenterToTargetXAngleRawDegrees));
			logData.AddData("RR:HiMidY", String.format("%.2f", publicTargetRawData.HighMiddleY));	
		}
		else {
			logData.AddData("RR:Camera",  "N/A");
			logData.AddData("RR:Angle",  "N/A");
			logData.AddData("RR:HiMidY", "N/A");	
		}
	}
	
	//=========================================================================
	//	Task Executed By Timer
	//=========================================================================	
 	
	// poll RoboRealm to read current values
 	public void update() { 
 		// get the Field Of View Dimensions
 		//_fovDimensions = _rrAPI.getDimension();
 	 	Vector vector;
 	 	RawImageData pollingThreadWorkingTargetRawData;
 	 	
 	    // get data from RoboRealm
 		// This must match what is in the config of the "Point Location" pipeline step in RoboRealm
 	    try {
 	    	vector = _rrAPI.getVariables("SW_X,SW_Y,SE_X,SE_Y,HI_MID_Y,SCREEN_WIDTH,SCREEN_HEIGHT,BLOB_COUNT,CamType");
 	    }
 	    catch(Exception ex) {
 	    	vector = null;
 	    }
 	    
 	    _callElapsedTimeMSec = new Date().getTime() - _lastCallTimestamp;
 
 	    if (vector==null) {
 	    	//Increment bad data counter 	    	
 			try {
 				if(lock.tryLock(MAX_LOCK_WAIT_TIME_MSEC, TimeUnit.MILLISECONDS)){
 					_pollingThreadPrivateTargetRawData.BadDataCounter += 1; 
 				}
 			} catch (InterruptedException e) {
 				e.printStackTrace();
 			}finally{
 				//release lock
 				lock.unlock();
 			}
 	    	
 	    	// write 1st 10 errors then every 50
 	    	if(_pollingThreadPrivateTargetRawData.BadDataCounter <= 10 
 	    			|| _pollingThreadPrivateTargetRawData.BadDataCounter % 50 == 0) {
 	    		System.out.println("Error in GetVariables, did not return any results [" 
 	    							+ _pollingThreadPrivateTargetRawData.BadDataCounter + "]");
 	    	}
 	    }
 	    else if(vector.size() == EXPECTED_ARRAY_SIZE) {
 	    	pollingThreadWorkingTargetRawData = new RawImageData();
 	    	
 	    	// parse the results and build the image data
 	    	pollingThreadWorkingTargetRawData.Timestamp = new Date().getTime();
 	    	pollingThreadWorkingTargetRawData.CameraType = (String)vector.elementAt(CAMERA_TYPE_IDX);	    	
 	    	pollingThreadWorkingTargetRawData.SouthWestX = Double.parseDouble((String)vector.elementAt(SOUTHWEST_X_IDX));
 	    	pollingThreadWorkingTargetRawData.SouthWestY = Double.parseDouble((String)vector.elementAt(SOUTHWEST_Y_IDX));
 	    	pollingThreadWorkingTargetRawData.SouthEastX = Double.parseDouble((String)vector.elementAt(SOUTHEAST_X_IDX));
 	    	pollingThreadWorkingTargetRawData.SouthEastY = Double.parseDouble((String)vector.elementAt(SOUTHEAST_Y_IDX));
 	    	pollingThreadWorkingTargetRawData.HighMiddleY = Double.parseDouble((String)vector.elementAt(HIGH_MIDDLE_Y_IDX));
 	    	
 	    	pollingThreadWorkingTargetRawData.BlobCount = Integer.parseInt((String)vector.elementAt(BLOB_COUNT_IDX));
 	    	
 	    	Dimension fovDimensions = new Dimension();
 	    	fovDimensions.width = Double.parseDouble((String)vector.elementAt(RAW_WIDTH_IDX));
 	    	fovDimensions.height = Double.parseDouble((String)vector.elementAt(RAW_HEIGHT_IDX));
 	    	pollingThreadWorkingTargetRawData.FOVDimensions = fovDimensions;
 	    	
 	    	//double estDistance = CalcDistanceUsingPolynomial(pollingThreadWorkingTargetRawData.HighMiddleY);
 	    	double estDistanceRaw = GeneralUtilities.RoundDouble(CalcDistanceUsingCameraAngle(pollingThreadWorkingTargetRawData.HighMiddleY), 2);
 	    	double estDistanceAdj = estDistanceRaw + BOILER_DISTANCE_OFFSET_INCHES;
 	    	
 	    	pollingThreadWorkingTargetRawData.EstimatedDistance = estDistanceAdj;
 	    	
 	    	pollingThreadWorkingTargetRawData.ResponseTimeMSec = _callElapsedTimeMSec; 	    	
 	
 	    	// calc the horiz center of the image
 	    	double fovXCenterPoint = fovDimensions.width / 2.0;
 	    	
 	    	// calc the target center point
 	    	double targetXCenterPoint = Math.round(((pollingThreadWorkingTargetRawData.SouthEastX 
 	    			+ pollingThreadWorkingTargetRawData.SouthWestX) / 2.0) + _cameraCalibrationFactor);
 	    	
 	    	double fovCenterToTargetXAngleRawDegrees = ((fovXCenterPoint - targetXCenterPoint) * CAMERA_FOV_HORIZONTAL_DEGREES) / fovDimensions.width;
 	    	
 	    	// round to 2 decimal places
 	    	fovCenterToTargetXAngleRawDegrees = Math.round(fovCenterToTargetXAngleRawDegrees * 100) / 100.0;	
 	    
 	    	pollingThreadWorkingTargetRawData.FovCenterToTargetXAngleRawDegrees = fovCenterToTargetXAngleRawDegrees + CAMERA_TO_SHOOTER_OFFSET_IN_DEGREES;
 	    	
 	    	pollingThreadWorkingTargetRawData.IsVisionDataValid = true;
 	    	
 	    	// limit spamming
 	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
	    		System.out.println("Vision Data Valid? " + pollingThreadWorkingTargetRawData.IsVisionDataValid
	    							+ " |Camera= " + pollingThreadWorkingTargetRawData.CameraType 
	    							+ " |Angle= " + pollingThreadWorkingTargetRawData.FovCenterToTargetXAngleRawDegrees
	    							+ " |HiMidY= " + pollingThreadWorkingTargetRawData.HighMiddleY
	    							+ " |DistInchesAdj= " + pollingThreadWorkingTargetRawData.EstimatedDistance
	    							+ " |mSec=" + pollingThreadWorkingTargetRawData.ResponseTimeMSec
	    							+ " |BlobCnt= " + pollingThreadWorkingTargetRawData.BlobCount
	    							+ " |DistInchesRaw= " + estDistanceRaw);
	    		// reset last time
	    		_lastDebugWriteTimeMSec = new Date().getTime();
 	    	}
 	    	
 	    	//Reset the counter 
 	    	pollingThreadWorkingTargetRawData.BadDataCounter = 0;
 	    	
 	    	// ================================================================================
 	    	// thread safe copy thread local to public holding lock as short a time as possible
 	    	// ================================================================================
 			try {
 				if(lock.tryLock(MAX_LOCK_WAIT_TIME_MSEC, TimeUnit.MILLISECONDS)){
 					// get current values from polling thread
 					_pollingThreadPrivateTargetRawData = new RawImageData();
 					_pollingThreadPrivateTargetRawData.BlobCount = pollingThreadWorkingTargetRawData.BlobCount;
 					_pollingThreadPrivateTargetRawData.CameraType = pollingThreadWorkingTargetRawData.CameraType;
 					_pollingThreadPrivateTargetRawData.EstimatedDistance = pollingThreadWorkingTargetRawData.EstimatedDistance;
 					_pollingThreadPrivateTargetRawData.FOVDimensions = pollingThreadWorkingTargetRawData.FOVDimensions;
 					_pollingThreadPrivateTargetRawData.HighMiddleY = pollingThreadWorkingTargetRawData.HighMiddleY;
 					_pollingThreadPrivateTargetRawData.ResponseTimeMSec = pollingThreadWorkingTargetRawData.ResponseTimeMSec;
 					_pollingThreadPrivateTargetRawData.SouthEastX = pollingThreadWorkingTargetRawData.SouthEastX;
 					_pollingThreadPrivateTargetRawData.SouthEastY = pollingThreadWorkingTargetRawData.SouthEastY;
 					_pollingThreadPrivateTargetRawData.SouthWestX = pollingThreadWorkingTargetRawData.SouthWestX;
 					_pollingThreadPrivateTargetRawData.SouthWestY = pollingThreadWorkingTargetRawData.SouthWestY;
 					_pollingThreadPrivateTargetRawData.Timestamp = pollingThreadWorkingTargetRawData.Timestamp;
 					_pollingThreadPrivateTargetRawData.FovCenterToTargetXAngleRawDegrees = pollingThreadWorkingTargetRawData.FovCenterToTargetXAngleRawDegrees;
 					_pollingThreadPrivateTargetRawData.IsVisionDataValid = pollingThreadWorkingTargetRawData.IsVisionDataValid;
 				}
 			} catch (InterruptedException e) {
 				e.printStackTrace();
 			}finally{
 				//release lock
 				lock.unlock();
 			}
 			
 	    	_lastCallTimestamp = new Date().getTime();
 	    } else {
 	    	//Increment bad data counter 	    	
 			try {
 				if(lock.tryLock(MAX_LOCK_WAIT_TIME_MSEC, TimeUnit.MILLISECONDS)){
 					_pollingThreadPrivateTargetRawData.BadDataCounter += 1; 
 				}
 			} catch (InterruptedException e) {
 				e.printStackTrace();
 			}finally{
 				//release lock
 				lock.unlock();
 			}
 			
 			//limitspamming
 	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
	 	    	System.out.println("Vision Data Recieved But Unexpected Array Size: " + vector.size());
	    		// reset last time
	    		_lastDebugWriteTimeMSec = new Date().getTime();
 	    	}
 	    }
 	} 
 	
 	// this method used a n order polynomial to estimate the distance based on data collected using a specific, fixed camera angle approx 40 degrees
 	//	the polynomial coefficients will not be correct for other angles
    private double CalcDistanceUsingPolynomial(double highMiddleYInPixels)
    {
    	//=((-1.724236*10^-7) * A26^3) + ((1.6430741*10^-4) * A26^2) - (0.06984136 * A26) + 18.45576757
    	//double estDistance =(-1.724236 * Math.pow(10, -7) * Math.pow(_newTargetRawData.HighMiddleY, 3)) 
    	//						+ (1.6430741 * Math.pow(10, -4) * Math.pow(_newTargetRawData.HighMiddleY, 2)) 
    	//						+ (-0.06984136 * _newTargetRawData.HighMiddleY) 
    	//						+ 18.45576757;
    	
    	// inches = -0.00005952 pixels^3 + 0.04060523pixels^2 - 10.34270244pixels + 963.84165834
    	//double estDistance = (-0.00005952 * Math.pow(_newTargetRawData.HighMiddleY, 3)) 
    	//						+ (0.04060523 * Math.pow(_newTargetRawData.HighMiddleY, 2))
    	//						+ (-10.34270244 * _newTargetRawData.HighMiddleY) 
    	//						+ 963.84165834;
    	
    	// charlie's original numbers to front of can
    	// inches= -0.00000207pixels^3 + 0.00209429pixels^2 - 0.91180787pixels + 235.66718419
    	//double estDistance = (-0.00000207 * Math.pow(_newTargetRawData.HighMiddleY, 3))
    	//						+ (0.00209429 * Math.pow(_newTargetRawData.HighMiddleY, 2))
    	//						+ (-0.91180787 * _newTargetRawData.HighMiddleY)
    	//						+ 235.66718419;
    	
    	// charlie's numbers to center of can
    	// inches= -0.00005952pixels^3 + 0.04198913pixels^2 - 10.982809pixels + 1046.4642
    	
    	RawImageData publicTargetRawData = get_newTargetRawData();
    	
    	double estDistanceInInches = (-0.000001 * Math.pow(publicTargetRawData.HighMiddleY, 3))
		    							+ (0.001388 * Math.pow(publicTargetRawData.HighMiddleY, 2))
		    							+ (-0.783982 * publicTargetRawData.HighMiddleY)
		    							+ 239.332871;
    	
    	return estDistanceInInches;
    }
 	
    // this method uses the actual camera angle to estimate the distance
    private double CalcDistanceUsingCameraAngle(double highMiddleYInPixels)
    {
    	final double HALF_FIELD_OF_VIEW_IN_DEGREES = 23.4425175; //23.5672121;			// 23.5645549; 
    	final double TARGET_HEIGHT_IN_INCHES = 87.875;					// 87.875; 	
    	final double SENSOR_HEIGHT_PIXELS = 472.273487; //477.511871;					// 479.115486; 
    	final double FOCAL_LENGTH_IN_INCHES = 0.14407609; //0.13916669;					// 0.14362022; 
    	final double OFFSET_0_IN_INCHES = 9.04908729;	//9.43578495;						// 9.99997139; 	
    	final double OFFSET_1_IN_INCHES = 0.17265693;	//0.13906082;						// 0.1409929;
    	
     	final double CAMERA_ANGLE_IN_DEGREES = 38.5;	//38.76;	// <====== SET ME BASED ON CALIB PROCEDURE ============


    	double sensorHeightInInches = Math.tan(Math.toRadians(HALF_FIELD_OF_VIEW_IN_DEGREES)) * FOCAL_LENGTH_IN_INCHES * 2.0;
    	
    	double cameraAngleInRadians = Math.toRadians(CAMERA_ANGLE_IN_DEGREES);
    	
    	double relativeTargetHeight = TARGET_HEIGHT_IN_INCHES 
    									- 16.965
    									+ (0.925 * Math.cos(Math.toRadians(19.75 + CAMERA_ANGLE_IN_DEGREES)));
    	
    	double x3 = (FOCAL_LENGTH_IN_INCHES * Math.tan(cameraAngleInRadians))
    					- (sensorHeightInInches / 2.0)
    					+ ((highMiddleYInPixels * sensorHeightInInches) / SENSOR_HEIGHT_PIXELS);
    	
    	double estDistanceInInches = (
    								  ( (FOCAL_LENGTH_IN_INCHES / (x3 * Math.pow(Math.cos(cameraAngleInRadians), 2)) - (Math.tan(cameraAngleInRadians)) )
    									* relativeTargetHeight
    									* (1 - OFFSET_1_IN_INCHES)
    									)
    									- OFFSET_0_IN_INCHES
    									+ (29.75 + (0.925 * Math.sin(Math.toRadians(19.75 + CAMERA_ANGLE_IN_DEGREES))))
    								  );
    	
    	return estDistanceInInches;
    }
    
    // this is the task run by the timer
    /*private class RoboRealmUpdater extends TimerTask { 
 		public void run() { 
 			while(true) { 
 				update(); 
 				
 				try { 
 					// sleep for 10 mSec
 					Thread.sleep(10); 
				} catch (InterruptedException e) { 
 					e.printStackTrace(); 
				} 
 			} 
 		} 
 	}*/
    
    // =========================================================
    // Property Accessors
    // =========================================================
 	public double get_Angle() { 
    	RawImageData publicTargetRawData = get_newTargetRawData();
 		return publicTargetRawData.FovCenterToTargetXAngleRawDegrees; 
 	}
    
    public double get_fovCenterToTargetXAngleRawDegrees() {
    	RawImageData publicTargetRawData = get_newTargetRawData();
 		return publicTargetRawData.FovCenterToTargetXAngleRawDegrees; 
    }
    
    public boolean get_isVisionDataValid() {
    	RawImageData publicTargetRawData = get_newTargetRawData();
    	
    	if(publicTargetRawData != null) {
    		return publicTargetRawData.IsVisionDataValid;
    	}
    	else {
    		return false;
    	}
    		
    }
    
    public double get_DistanceToBoilerInches() {
    	RawImageData publicTargetRawData = get_newTargetRawData();
    	
		if (get_isVisionDataValid()) {
    		return publicTargetRawData.EstimatedDistance;
    	}
    	else {
    		return 10.0;
    	}
    }
    
    // returns true if the robot is close enough to the gear peg using vision
    public boolean get_isInGearHangPosition() {
    	RawImageData publicTargetRawData = get_newTargetRawData() ;
    	
		if (get_isVisionDataValid()
				&& (publicTargetRawData.SouthEastY > TARGET_MINIMUM_Y_VALUE) 
				&& (publicTargetRawData.SouthWestY > TARGET_MINIMUM_Y_VALUE)) {
			return true;
		} else {
			return false;
		}
    }
    
    // uses a lock to safely get a copy of the 
    public RawImageData get_newTargetRawData() {
		try {
			if(lock.tryLock(MAX_LOCK_WAIT_TIME_MSEC, TimeUnit.MILLISECONDS)){
				// get current values from polling thread
				_pollingThreadPublicTargetRawData = new RawImageData();
				_pollingThreadPublicTargetRawData.BlobCount = _pollingThreadPrivateTargetRawData.BlobCount;
				_pollingThreadPublicTargetRawData.CameraType = _pollingThreadPrivateTargetRawData.CameraType;
				_pollingThreadPublicTargetRawData.EstimatedDistance = _pollingThreadPrivateTargetRawData.EstimatedDistance;
				_pollingThreadPublicTargetRawData.FOVDimensions = _pollingThreadPrivateTargetRawData.FOVDimensions;
				_pollingThreadPublicTargetRawData.HighMiddleY = _pollingThreadPrivateTargetRawData.HighMiddleY;
				_pollingThreadPublicTargetRawData.ResponseTimeMSec = _pollingThreadPrivateTargetRawData.ResponseTimeMSec;
				_pollingThreadPublicTargetRawData.SouthEastX = _pollingThreadPrivateTargetRawData.SouthEastX;
				_pollingThreadPublicTargetRawData.SouthEastY = _pollingThreadPrivateTargetRawData.SouthEastY;
				_pollingThreadPublicTargetRawData.SouthWestX = _pollingThreadPrivateTargetRawData.SouthWestX;
				_pollingThreadPublicTargetRawData.SouthWestY = _pollingThreadPrivateTargetRawData.SouthWestY;
				_pollingThreadPublicTargetRawData.Timestamp = _pollingThreadPrivateTargetRawData.Timestamp;
				_pollingThreadPublicTargetRawData.FovCenterToTargetXAngleRawDegrees = _pollingThreadPrivateTargetRawData.FovCenterToTargetXAngleRawDegrees;
				_pollingThreadPublicTargetRawData.IsVisionDataValid = _pollingThreadPrivateTargetRawData.IsVisionDataValid;
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}finally{
			//release lock
			lock.unlock();
		}
		    	
		return _pollingThreadPublicTargetRawData;
    }
    
    //================================================================================================
	private Runnable RoboRealmUpdater = new Runnable() {
		@Override
		public void run() {   
            
            // =============================================================================
            // start looping
            // =============================================================================
            while(!Thread.interrupted()) {            	
            	update();
            	        		
        		// sleep each cycle to avoid Robot Code not updating often enough issues
        		try {
					Thread.sleep(POLLING_CYCLE_IN_MSEC);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
            }
	            	
		}
	};
}