package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Center Peg And Shoot" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	6.Mar.2017	Initial Version
//1			TomB		29.Mar.2017	Implemented support for Alliance selection to determine Gyro Turn Angle
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangCenterGearAndShoot {
	// define class level variables for Robot subsystems
	private AutoShootController _autoShootController;
	private GearHandler _gearHandler;
	private ChassisAutoAimController _autoAim;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private HangGearController _hangGearController;
	private ALLIANCE_COLOR _allianceColor;
	
	private double _gyroTurnTargetAngle;
	private static final double RED_ALLIANCE_GYRO_TARGET_TURN_ANGLE = -40.0;
	private static final double BLUE_ALLIANCE_GYRO_TARGET_TURN_ANGLE = 40.0;

	private int _targetShootingDistanceInInches;
	private static final int RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 140;
	private static final int BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 145;

	private enum AUTON_STATE {
		UNDEFINED, 
		GOTTA_GO_FAST,
		RUN_GEAR_SEQUENCE_AND_MOVE_BACK,
		GYRO_TURN,
		VISION_TURN,
		SHOOT,
		FINISHED
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangCenterGearAndShoot(AutoShootController autoShoot, GearHandler gearHandler, ChassisAutoAimController autoAim,  
			HangGearController hangGear, Shooter shooter, TrajectoryDriveController trajController, ALLIANCE_COLOR allianceColor) {
		// these are the subsystems that this auton routine needs to control
		_autoShootController = autoShoot;
		_gearHandler = gearHandler;
		_hangGearController = hangGear;
		_shooter = shooter;
		_autoAim = autoAim;
		_trajController = trajController;
		_allianceColor = allianceColor;
		
		// determine Fixed Gyro Turn Angle based on Alliance Color
		switch(_allianceColor) {
			case BLUE_ALLIANCE:
				_gyroTurnTargetAngle = BLUE_ALLIANCE_GYRO_TARGET_TURN_ANGLE;
				_targetShootingDistanceInInches = BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES;
				break;
				
			case RED_ALLIANCE:
				_gyroTurnTargetAngle = RED_ALLIANCE_GYRO_TARGET_TURN_ANGLE;
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
		_autonState = AUTON_STATE.GOTTA_GO_FAST;
		_autoShootController.LoadTargetDistanceInInches(_targetShootingDistanceInInches);
		
		_autoShootController.StopShooter();
		
		_trajController.configureIsHighGear(false);
		_trajController.loadProfile(MOTION_PROFILE.CENTER_GEAR, false);
		_trajController.enable();
		
		// chg vision camera to Boiler
		_autoShootController.EnableBoilerCam();
		
		DriverStation.reportWarning(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangCenterGear Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() { 	
      	switch (_autonState) {
      		case GOTTA_GO_FAST:
      			if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		DriverStation.reportWarning("Gear Tilt Zero completed!", false);
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
      			
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				if(_allianceColor == ALLIANCE_COLOR.BLUE_ALLIANCE) {
      					_trajController.loadProfile(MOTION_PROFILE.J_TURN, true);
      				} else {
      					_trajController.loadProfile(MOTION_PROFILE.J_TURN, false);
      				}
      				_trajController.enable();
      				DriverStation.reportWarning(Double.toString(_trajController.getCurrentHeading()), false);
      				_hangGearController.Initialize();
      				
      				// chg state
      				_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE_AND_MOVE_BACK;
      				DriverStation.reportWarning("===> Chg state from MOVE_TO_TARGET to RUN_GEAR_SEQUENCE", false);
      			}
      			break;
      			
      		case RUN_GEAR_SEQUENCE_AND_MOVE_BACK:
      			if (_trajController.getCurrentSegment() >= 50) {
      				_autoShootController.RunShooterAtTargetSpeed();
      			}

      			boolean isStillRunning = _hangGearController.ExecuteRentrant();
      			
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				// chg vision camera to Boiler
      				_autoShootController.EnableBoilerCam();
      				
      				// chg state
      				_autonState = AUTON_STATE.VISION_TURN;
      				DriverStation.reportWarning("===> Chg state from RUN_GEAR_SEQUENCE to MOVE_BACK", false);
      			}
      			break;
      			
      		case GYRO_TURN:
      			_autoShootController.RunShooterAtTargetSpeed();
      			
      			// call turn controller
      			_autoAim.motionMagicMoveToTarget(_gyroTurnTargetAngle);
      			
      			// have we reached the target angle w/i the threshhold ?
      			if (Math.abs(_autoAim.currentHeading()) > 25.0) {
      				
      				// chg state
      				_autonState = AUTON_STATE.VISION_TURN;
      				DriverStation.reportWarning("===> Chg state from GYRO_TURN to VISION_TURN", false);
      			}
      			break;
      			
      		case VISION_TURN:
      			//_autoShootController.LoadTargetDistanceUsingVision();
      			_autoShootController.RunShooterAtTargetSpeed();
      			_autoShootController.AimWithVision(0.0);			// Aim robot to the boiler
      			
      			if(_autoShootController.IsReadyToShoot()) {
      				// start shooter feeder motors
      				_shooter.ToggleRunShooterFeeder();
      				
      				// chg state
      				_autonState = AUTON_STATE.SHOOT;
      				DriverStation.reportWarning("PEW PEW PEW PEW PEW", false);
      			}
      			
      			break;
      			
      		case SHOOT:
      			//_autoShootController.LoadTargetDistanceUsingVision();
      			_autoShootController.RunShooterAtTargetSpeed();
      			//_autoShootController.AimWithVision();
      			_autoShootController.ChassisFullStop();
      			// start shooter feeder motors reentrant function
      			_shooter.RunShooterFeederReentrant();
      			break;
      			
      		case FINISHED:
      			_isStillRunning = false;
      			break;
      			
      		case UNDEFINED:
      			break;
      	}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed HangCenterGear Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	public void Disabled() {
		_trajController.disable();
		_trajController.stopTrajectoryController();
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() {
		return _isStillRunning;
	}
}