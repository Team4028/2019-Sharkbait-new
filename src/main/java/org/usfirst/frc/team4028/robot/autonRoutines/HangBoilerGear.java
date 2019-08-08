package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Boiler Side Peg" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//1.0 		Sebas 		4.Mar.2017	Added Motion Profile + Hang Gear
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangBoilerGear {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private TrajectoryDriveController _trajController;
	private HangGearController _hangGearController;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_TARGET,
		RUN_GEAR_SEQUENCE
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangBoilerGear(GearHandler gearHandler, HangGearController hangGear, TrajectoryDriveController trajController) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_hangGearController = hangGear;
		_trajController = trajController;
		DriverStation.reportWarning("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		_autonState = AUTON_STATE.MOVE_TO_TARGET;
		
		_trajController.configureIsHighGear(false);
		_trajController.loadProfile(MOTION_PROFILE.BOILER_GEAR, false);
		_trajController.enable();
		DriverStation.reportWarning(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangBoilerSideGear Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
		if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	} else {
    		DriverStation.reportWarning("Zeroed", false);
    		_gearHandler.MoveGearToScorePosition();
    	}
      	
      	switch (_autonState) {
      		case MOVE_TO_TARGET:
      			if (_trajController.getCurrentSegment() == 140) {
      				_trajController.isVisionTrackingEnabled(true);
      			}
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				_trajController.isVisionTrackingEnabled(false);
      				_hangGearController.Initialize();
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE;
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE:
      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			if (!isStillRunning) {
      				DriverStation.reportWarning("Done", false);
      			}
      			break;
      			
      		case UNDEFINED:
      			break;
      	}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed HangBoilerGear Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	public void Disabled() {
		_trajController.disable();
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() {
		return _isStillRunning;
	}
}