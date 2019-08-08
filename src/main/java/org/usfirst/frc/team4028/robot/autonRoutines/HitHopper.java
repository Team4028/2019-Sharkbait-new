package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE_COLOR;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.controllers.AutoShootController;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

// this class implements the logic for the "Hit the Hopper and Shoot" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	7.Mar.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class HitHopper {
	// define class level variables for Robot subsystems
	private AutoShootController _autoShootController;
	private Chassis _chassis;
	private ChassisAutoAimController _autoAim;
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private TrajectoryDriveController _trajController;
	private ALLIANCE_COLOR _allianceColor;
	
	private int _targetShootingDistanceInInches;
	private static final int RED_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 124;
	private static final int BLUE_BOILER_TARGET_SHOOTING_DISTANCE_IN_INCHES = 163;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_BOILER_HELLA_FAST_X,
		WAIT,
		MOVE_TO_SHOOTING_POSITION,
		VISION_TURN,
		SHOOT
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private long _waitStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	private static final int WAIT_TIME_MSEC = 1100;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HitHopper(AutoShootController autoShoot, Chassis chassis, ChassisAutoAimController autoAim, GearHandler gearHandler, Shooter shooter, TrajectoryDriveController trajController, ALLIANCE_COLOR allianceColor) {
		// these are the subsystems that this auton routine needs to control
		_autoShootController = autoShoot;
		_autoAim = autoAim;
		_chassis = chassis;
		_gearHandler = gearHandler;
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
		_autonState = AUTON_STATE.MOVE_TO_BOILER_HELLA_FAST_X;
		_autoShootController.LoadTargetDistanceInInches(_targetShootingDistanceInInches);
		_autoShootController.StopShooter();
		
		_trajController.configureIsHighGear(false);
		switch(_allianceColor) {
			case BLUE_ALLIANCE:
				_trajController.loadProfile(MOTION_PROFILE.MOVE_TO_HOPPER_BLUE_X, true);
				break;
				
			case RED_ALLIANCE:
				_trajController.loadProfile(MOTION_PROFILE.MOVE_TO_HOPPER_RED_X, false);
				break;
		}
		_trajController.enable();
		
		_autoShootController.EnableBoilerCam();
		
		DriverStation.reportWarning("===== Entering Hit Hopper Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		switch(_autonState) {
			case MOVE_TO_BOILER_HELLA_FAST_X:
				if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      	      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      	      		//			we must treat it as a Reentrant function
      	      		//			and automatically recall it until complete
      	    		_gearHandler.ZeroGearTiltAxisReentrant();
      	    	} else {
      	    		DriverStation.reportWarning("Gear Tilt Zero completed!", false);
      	    		_gearHandler.MoveGearToScorePosition();
      	    	}
				
				if(_trajController.onTarget()) {
					_trajController.disable();
					_waitStartedTimeStamp = System.currentTimeMillis();
					DriverStation.reportWarning("Starting to Wait", false);
					_autonState = AUTON_STATE.WAIT;
				}
				break;
				
			case WAIT:
				_chassis.ArcadeDrive(-0.35, 0.0);
				
				if((System.currentTimeMillis() - _waitStartedTimeStamp) > 800) {
					_autoShootController.RunShooterAtTargetSpeed();
				}
				
				if((System.currentTimeMillis() - _waitStartedTimeStamp) > WAIT_TIME_MSEC) {
					switch(_allianceColor) {
						case BLUE_ALLIANCE:
							_trajController.loadProfile(MOTION_PROFILE.TWO_GEAR_SHORT_FWD, true);
							break;
							
						case RED_ALLIANCE:
							_trajController.loadProfile(MOTION_PROFILE.TWO_GEAR_SHORT_FWD, false);
							break;
					}
					_trajController.enable();
					_autonState = AUTON_STATE.MOVE_TO_SHOOTING_POSITION;
					DriverStation.reportWarning("Moving to Shoot", false);
				}
				break;
				
			case MOVE_TO_SHOOTING_POSITION:
				_autoShootController.RunShooterAtTargetSpeed();
				if (_trajController.onTarget()) {
					_trajController.disable();
					DriverStation.reportWarning("Moved To Shooting Position", false);
					_autonState = AUTON_STATE.VISION_TURN;
				}
				break;
				
			case VISION_TURN:
				_autoShootController.AimWithVision(0.0);
				
				// need to keep calling this so is motor at speed gets updated so IsReadyToShoot can pass
				_autoShootController.LoadTargetDistanceUsingVision();
				_autoShootController.RunShooterAtTargetSpeed();
				
      			if(_autoShootController.IsReadyToShoot()) {
      				// start shooter feeder motors
      				_shooter.ToggleRunShooterFeeder();
      				
      				// chg state
      				_autonState = AUTON_STATE.SHOOT;
      				DriverStation.reportWarning("PEW PEW PEW PEW PEW", false);
      			}
				break;
				
			case SHOOT:
				//_autoShootController.AimWithVision(0);
				_shooter.RunShooterFeederReentrant();
				_gearHandler.FlapAnnReentrant();
				
			case UNDEFINED:
				break;
		}
		
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Complete Hit Hopper Auton =====", false);
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