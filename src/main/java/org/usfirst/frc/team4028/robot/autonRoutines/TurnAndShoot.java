package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Turn and shoot into the Boiler" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//1.0 		Sebas 		6.Mar.2017	Added Motion Profile
//------------------------------------------------------
//=====> For Changes see Sebas
public class TurnAndShoot {
	// define class level variables for Robot subsystems
	private AutoShootController _autoShoot;
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private enum AUTON_STATE {
		UNDEFINED, 
		MOVE_TO_BASELINE,
		AIM_WITH_VISION,
		SHOOT
	}
	
	private AUTON_STATE _autonState;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public TurnAndShoot(AutoShootController autoShoot, GearHandler gearHandler, Shooter shooter, TrajectoryDriveController trajController) {
		// these are the subsystems that this auton routine needs to control
		_autoShoot = autoShoot;
		_gearHandler = gearHandler;
		_shooter = shooter;
		_trajController = trajController;
		DriverStation.reportError("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		
		_autonState = AUTON_STATE.MOVE_TO_BASELINE;
		
		_autoShoot.LoadTargetDistanceInInches(100);
		
		_trajController.configureIsHighGear(false);
		_trajController.loadProfile(MOTION_PROFILE.CENTER_GEAR, false);
		_trajController.enable();
		DriverStation.reportWarning("===== Entering TurnAndShoot Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
      	switch (_autonState) {
      		case MOVE_TO_BASELINE:
      			if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
      			
      			if (_trajController.getCurrentSegment() > 70) {
      				_autoShoot.RunShooterAtTargetSpeed();
      			}
      			
      			if(_trajController.onTarget()) {
      				_trajController.disable();
      				_autonState = AUTON_STATE.AIM_WITH_VISION;
      			}
      			break;
      			
      		case AIM_WITH_VISION:
      			_autoShoot.AimWithVision(0);
      			
      			if (_autoShoot.IsReadyToShoot()) {
      				_shooter.ToggleRunShooterFeeder();
      				
      				_autonState = AUTON_STATE.SHOOT;
      			}
      			break;
      			
      		case SHOOT:
      			_shooter.RunShooterFeederReentrant();
      			break;
      	}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed TurnAndShoot Auton =====", false);
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