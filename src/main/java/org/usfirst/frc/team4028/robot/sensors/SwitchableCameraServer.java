package org.usfirst.frc.team4028.robot.sensors;

import java.nio.file.Files;
import java.nio.file.Paths;

import org.opencv.core.Mat;
import org.usfirst.frc.team4028.robot.utilities.CircularQueue;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for operator/driver cameras
//=====> For Changes see Nick Donahue (javadotmakeitwork)
//=========================REVISION BLOCK==================================
//Rev		By			D/T				Description
// 0		Nick		2/27 8:47		Added 4th Camera and Added "Swapped to Next" method for increased organization
// 1		TomB		5.Mar.2017		Added code to handle unplugging & replugging cameras
// 2        Nick        9.Mar.2017      Fixed the Issue of not Knowing Which Camera is Which via sendable chooser   
//-------------------------------------------------------------
public class SwitchableCameraServer {	
	// working variables
	private String _cameraName;
	private String _previousCameraName;
	private String _gearCameraName;
	private String _shooterCameraName;
	private String _climberCameraName;
	private String _driverCameraName;
	
	private boolean _isCam0Present;
	private boolean _isCam1Present;
	private boolean _isCam2Present;
	private boolean _isCam3Present;
	
	private CircularQueue<String> _camList;

	//============================================================================================
	// constructors follow
	//============================================================================================
	public SwitchableCameraServer(String cameraname) {	
		// check what cameras are plugged in
    	_isCam0Present = Files.exists(Paths.get("/dev/video0"));
    	_isCam1Present = Files.exists(Paths.get("/dev/video1"));
    	_isCam2Present = Files.exists(Paths.get("/dev/video2"));
    	_isCam3Present = Files.exists(Paths.get("/dev/viedo3"));
    	
    	/*System.out.println("Cam0isConnected:" + isCam0Present);
    	System.out.println("Cam1isConnected:" + isCam1Present);
    	System.out.println("Cam2isConnected:" + isCam2Present);
    	System.out.println("Cam3isConnected:" + isCam3Present);*/
    	
    	DriverStation.reportWarning("Cam0 is Connected? " + _isCam0Present, false);
    	DriverStation.reportWarning("Cam1 is Connected? " + _isCam1Present, false);
    	DriverStation.reportWarning("Cam2 is Connected? " + _isCam2Present, false);
    	DriverStation.reportWarning("Cam3 is Connected? " + _isCam3Present, false);
		
    	// build list of available cameras
		_camList = new CircularQueue<String>();
		
		if(_isCam0Present)	{ _camList.add("cam0"); }
		if(_isCam1Present)	{ _camList.add("cam1"); }
		if(_isCam2Present)	{ _camList.add("cam2"); }
		if(_isCam3Present)	{ _camList.add("cam3"); }
		
		// chg to the requested camera
		ChgToCamera(cameraname);
		
		// start the camera thread
		Thread cameraThread = new Thread(JavadotMakeItWork);
		cameraThread.start();
	}

	//============================================================================================
	// Methods follow
	//============================================================================================
	public void ChgToCamera(String cameraName) {
		if(_previousCameraName != cameraName) {
			// make sure the requested camera is available
			if(_camList.contains(cameraName)) {
				_cameraName = cameraName;
			} else {
				System.out.println("Camera not available: " + cameraName);
				DriverStation.reportError("Camera not available: " + _cameraName, false);
			}
		} else {
			System.out.println("Requested Camera is already current: " + cameraName);
		}
	}
	
	public void ChgToNextCamera() {		
		if(!_camList.isEmpty()) {
			ChgToCamera(_camList.getNext());
		} else {
			DriverStation.reportError("No Cameras Available", false);
		}
	}
	
	public void OutputToSmartDashboard(){
		SmartDashboard.putString("IsCam0Present:", Boolean.toString(_isCam0Present) + " (" + getCameraFunction("cam0") + ")");
		SmartDashboard.putString("IsCam1Present?", Boolean.toString(_isCam1Present) + " (" + getCameraFunction("cam1") + ")");
		SmartDashboard.putString("IsCam2Present?", Boolean.toString(_isCam2Present) + " (" + getCameraFunction("cam2") + ")");
		SmartDashboard.putString("IsCam3Present?", Boolean.toString(_isCam3Present) + " (" + getCameraFunction("cam3") + ")");
		if (_cameraName == null) {
			_cameraName = "N/A";
		}

		SmartDashboard.putString("CurrentCameraName:", _cameraName);
	}
	
	private String getCameraFunction(String cameraName) {
		if(_gearCameraName == cameraName) {
			return "Gear";
		}
		else if(_shooterCameraName == cameraName) {
			return "Shooter";
		}
		else if(_climberCameraName == cameraName) {
			return "Climber";
		}
		else if(_driverCameraName == cameraName) {
			return "Driver";
		} else {
			return "N/A";
		}
	}
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	public String getCurrentCameraName() {
		return _cameraName;
	}
	
	public void setGearCameraName(String cameraname) {
		_gearCameraName = cameraname;
	}
	
	public void setClimberCameraName(String cameraname) {
		_climberCameraName = cameraname;
	}
	
	public void setShooterCameraName(String cameraname) {
		_shooterCameraName = cameraname;
	}
	
	public void setDriverCameraName(String cameraname) {
		_driverCameraName = cameraname;
	}
	
	//============================================================================================
	// Task that is run on a separate thread
	//============================================================================================
	private Runnable JavadotMakeItWork = new Runnable() {
		@Override
		public void run() {   
	    	UsbCamera cam0 = null;
	    	UsbCamera cam1 = null;
	    	UsbCamera cam2 = null;
	    	UsbCamera cam3 = null;
	    	
	    	CvSink cvSink0 = null;
	    	CvSink cvSink1 = null;
	    	CvSink cvSink2 = null;
	    	CvSink cvSink3 = null;
	    	
			// create instances of the all the cameras
            // create sinks for each camera
	    	if(_isCam0Present) {
	    		cam0 = new UsbCamera("cam0", 0); 		
	    		//cam0.setFPS(20);
           		//cam0.setResolution(320, 240);
           		//cam0.setExposureAuto();
	    		cvSink0 = CameraServer.getInstance().getVideo(cam0); 
	    		cvSink0.setEnabled(false);
	    	}
	    	
	    	if(_isCam1Present) {
	    		cam1 = new UsbCamera("cam1",  1); 	
	    		//cam1.setFPS(20);
           		//cam1.setResolution(320, 240);
           		//cam1.setExposureAuto();
	    		cvSink1 = CameraServer.getInstance().getVideo(cam1);
	    		cvSink1.setEnabled(false);
	    	}
	    	
		    if(_isCam2Present) {
		    	cam2 = new UsbCamera("cam2", 2); 
		    	//cam2.setFPS(20);
           		//cam2.setResolution(320, 240);
           		//cam2.setExposureAuto();
		    	cvSink2 = CameraServer.getInstance().getVideo(cam2);
		    	cvSink2.setEnabled(false);
		    }
		    
		    if(_isCam3Present) {
		    	cam3 = new UsbCamera("cam3", 3); 
		    	//cam3.setFPS(20);
           		//cam3.setResolution(320, 240);
           		//cam3.setExposureAuto();
		    	cvSink3 = CameraServer.getInstance().getVideo(cam3);
		    	cvSink3.setEnabled(false);
		    }
 
            // create an output stream
            CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);
            
            // create a 2d array to hold the captured image
            Mat image = new Mat();
            
            // create a MjpegServer on port 1181
            //MjpegServer server = new MjpegServer("server", 1181);
            
            // set the image source for the mjepg server to be the output stream
            //server.setSource(outputStream);
            CameraServer.getInstance().getServer().setSource(outputStream);
            
            boolean isImageAvailable = false;
            boolean isCameraAvailableLastScan = false;
            
            // =============================================================================
            // start looping
            // =============================================================================
            while(!Thread.interrupted()) {            	
            	isImageAvailable = false;
            	
        		// enable this camera & configure it
        		if(_cameraName == "cam0") {              
        			// make sure camera is still plugged in
        			_isCam0Present = Files.exists(Paths.get("/dev/video0"));
        			
        			if (_isCam0Present) {
	          			try {        
	          				// if camera was jut swapped or unplugged & plugged back in
		        			if(_previousCameraName != _cameraName || !isCameraAvailableLastScan) {                		
		        				// disable the other cameras
		                		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
		                		//			you want to avoid USB bus overload!
		            			if(cvSink1 != null) { cvSink1.setEnabled(false); }
		            			
		            			if(cvSink2 != null) { cvSink2.setEnabled(false); }
		            			
		            			if(cvSink3 != null) { cvSink3.setEnabled(false); }
		            			
				                cvSink0.setEnabled(true);
			                	cam0.setFPS(20);
			               		cam0.setResolution(320, 240);
			               		//cam0.setExposureAuto();
			               		//cam0.setExposureManual(36);
			               		_previousCameraName = _cameraName;
			               		
			               		System.out.println("Camera Swapped to: " + _cameraName);
		        			}
		               		
		        			// grab the current frame from this camera and put it into the 2D array
		            		cvSink0.grabFrameNoTimeout(image);
		            		isImageAvailable = true;
		            		isCameraAvailableLastScan = true;
	        			} catch(Exception ex) { 
	        				isCameraAvailableLastScan = false;
	        				System.out.println("Camera Unplugged: " + _cameraName);
	        				DriverStation.reportError("Camera Unplugged: " + _cameraName, true);
	        			}
        			}
            	} 
        		// ===== CAM1 =======
        		else if(_cameraName == "cam1") {              
        			// make sure camera is still plugged in
        			_isCam1Present = Files.exists(Paths.get("/dev/video1"));
        			
        			if (_isCam1Present) {
	          			try {        
	          				// if camera was jut swapped or unplugged & plugged back in
		        			if (_previousCameraName != _cameraName || !isCameraAvailableLastScan) {                		
		        				// disable the other cameras
		                		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
		                		//			you want to avoid USB bus overload!
		            			if(cvSink0 != null) { cvSink0.setEnabled(false); }
		            			
		            			if(cvSink2 != null) { cvSink2.setEnabled(false); }
		            			
		            			if(cvSink3 != null) { cvSink3.setEnabled(false); }
		            			
		            			
				                cvSink1.setEnabled(true);
			                	cam1.setFPS(20);
			                	cam1.setResolution(320, 240);
			                	//cam1.setExposureAuto();
			                	//cam1.setExposureManual(36);
			               		_previousCameraName = _cameraName;
			               		
			               		System.out.println("Camera Swapped to: " + _cameraName);
		        			}
		               		
		        			// grab the current frame from this camera and put it into the 2D array
		            		cvSink1.grabFrameNoTimeout(image);
		            		isImageAvailable = true;
		            		isCameraAvailableLastScan = true;
	        			} catch(Exception ex) { 
	        				isCameraAvailableLastScan = false;
	        				System.out.println("Camera Unplugged: " + _cameraName);
	        			}
        			}
            	} 
        		// ===== CAM2 =======
        		else if(_cameraName == "cam2") {              
        			// make sure camera is still plugged in
        			_isCam2Present = Files.exists(Paths.get("/dev/video2"));
        			
        			if (_isCam2Present) {
	          			try {        
	          				// if camera was jut swapped or unplugged & plugged back in
		        			if(_previousCameraName != _cameraName || !isCameraAvailableLastScan) {                		
		        				// disable the other cameras
		                		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
		                		//			you want to avoid USB bus overload!
		            			if(cvSink0 != null) { cvSink0.setEnabled(false); }
		            			
		            			if(cvSink1 != null) { cvSink1.setEnabled(false); }
		            			
		            			if(cvSink3 != null) { cvSink3.setEnabled(false); }
		            			
				                cvSink2.setEnabled(true);
			                	cam2.setFPS(20);
			                	cam2.setResolution(320, 240);
			                	//cam2.setExposureAuto();
			                	//cam2.setExposureManual(36);
			               		_previousCameraName = _cameraName;
			               		
			               		System.out.println("Camera Swapped to: " + _cameraName);
		        			}
		               		
		        			// grab the current frame from this camera and put it into the 2D array
		            		cvSink2.grabFrameNoTimeout(image);
		            		isImageAvailable = true;
		            		isCameraAvailableLastScan = true;
	        			} catch(Exception ex) { 
	        				isCameraAvailableLastScan = false;
	        				System.out.println("Camera Unplugged: " + _cameraName);
	        			}
        			}
            	} 
        		// ===== CAM3 =======
        		else if(_cameraName == "cam3") {              
        			// make sure camera is still plugged in
        			_isCam3Present = Files.exists(Paths.get("/dev/video3"));
        			
        			if (_isCam3Present) {
	          			try {        
	          				// if camera was jut swapped or unplugged & plugged back in
		        			if(_previousCameraName != _cameraName || !isCameraAvailableLastScan) {                		
		        				// disable the other cameras
		                		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
		                		//			you want to avoid USB bus overload!
		            			if(cvSink0 != null) { cvSink0.setEnabled(false); }
		            			
		            			if(cvSink1 != null) { cvSink1.setEnabled(false); }
		            			
		            			if(cvSink2 != null) { cvSink2.setEnabled(false); }
		            			
				                cvSink3.setEnabled(true);
			                	cam3.setFPS(20);
			                	cam3.setResolution(320, 240);
			                	cam3.setExposureAuto();
			                	//cam3.setExposureManual(36);
			               		_previousCameraName = _cameraName;
			               		
			               		System.out.println("Camera Swapped to: " + _cameraName);
		        			}
		               		
		        			// grab the current frame from this camera and put it into the 2D array
		            		cvSink3.grabFrameNoTimeout(image);
		            		isImageAvailable = true;
		            		isCameraAvailableLastScan = true;
	        			} catch(Exception ex) { 
	        				isCameraAvailableLastScan = false;
	        				System.out.println("Camera Unplugged: " + _cameraName);
	        			}
        			}
            	} 
        		            	
        		if(isImageAvailable) {
		           	// push the captured frame to the output stream
	           		outputStream.putFrame(image);
        		}
        		
        		// sleep each cycle to avoid Robot Code not updating often enough issues
        		try {
					Thread.sleep(50); // was 20, 50mS = 20 frames / sec
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
            }
	            	
		}
	};
}