package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Boiler Side Peg and then Shoot" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	6.Mar.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangBoilerGearAndShoot {
	// define class level variables for Robot subsystems
	private AutoShootController _autoShootController;
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private HangGearController _hangGearController;
	private ALLIANCE_COLOR _allianceColor;
	
	private double _targetShootingDistanceInInches;
	private static final int RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 150;
	private static final int BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 150;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_TARGET,
		RUN_GEAR_SEQUENCE,
		VISION_TURN,
		SHOOT
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangBoilerGearAndShoot(AutoShootController autoShoot, GearHandler gearHandler, HangGearController hangGear, 
			Shooter shooter, TrajectoryDriveController trajController, ALLIANCE_COLOR allianceColor) {
		// these are the subsystems that this auton routine needs to control
		_autoShootController = autoShoot;
		_gearHandler = gearHandler;
		_hangGearController = hangGear;
		_shooter = shooter;
		_trajController = trajController;
		_allianceColor = allianceColor;
		
		switch(_allianceColor) {
		case BLUE_ALLIANCE:
			_targetShootingDistanceInInches = BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES;
			break;
			
		case RED_ALLIANCE:
			_targetShootingDistanceInInches = RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES;
			break;
		}
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
		switch(_allianceColor) {
		case BLUE_ALLIANCE:
			_trajController.loadProfile(MOTION_PROFILE.BOILER_GEAR, true);
			break;
			
		case RED_ALLIANCE:
			_trajController.loadProfile(MOTION_PROFILE.BOILER_GEAR, false);
			break;
		}
		_trajController.enable();
		DriverStation.reportWarning(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangBoilerSideGear Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {	
      	switch (_autonState) {
      		case MOVE_TO_TARGET:
      			if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		DriverStation.reportWarning("Zeroed", false);
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
      			if (_trajController.getCurrentSegment() == 140) {
      				_trajController.isVisionTrackingEnabled(true);
      			}
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				_trajController.isVisionTrackingEnabled(false);
      				DriverStation.reportWarning(Double.toString(_trajController.getCurrentHeading()), false);
      				_hangGearController.Initialize();
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE;
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE:
      			_autoShootController.RunShooterAtTargetSpeed();
      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			if (!isStillRunning) {
      				_autonState = AUTON_STATE.VISION_TURN;
      				DriverStation.reportWarning("===> Chg state from RUN_GEAR_SEQUENCE to VISION", false);
      			}
      			break;
      			
      		case VISION_TURN:
      			_autoShootController.AimWithVision(0);
      			
      			if(_autoShootController.IsReadyToShoot()) {
      				// start shooter feeder motors
      				_shooter.ToggleRunShooterFeeder();
      				
      				// chg state
      				_autonState = AUTON_STATE.SHOOT;
      				DriverStation.reportWarning("PEW PEW PEW PEW PEW", false);
      			}
      			break;
      			
      		case SHOOT:
      			_shooter.RunShooterFeederReentrant();
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