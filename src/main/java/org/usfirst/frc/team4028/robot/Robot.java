package org.usfirst.frc.team4028.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.AutonExecuter;
import org.usfirst.frc.team4028.robot.autonRoutines.*;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.*;
import org.usfirst.frc.team4028.robot.controllers.*;
import org.usfirst.frc.team4028.robot.sensors.*;
import org.usfirst.frc.team4028.robot.subsystems.*;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.utilities.DataLogger;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.utilities.MovingAverage;
import org.usfirst.frc.team4028.robot.utilities.ShooterTable;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The is the main code for:
 * 	    Team:	4028 "The Beak Squad"
 * 		Season: FRC 2017 "First Steamworks"
 * 		Robot:	Competition Chassis
 */
public class Robot extends IterativeRobot {
	// this value is printed on the Driver's Station message window on startup
	private static final String ROBOT_NAME = "COMPETITION Chassis";
	
	// ===========================================================
	//   Define class level instance variables for Robot Runtime objects & get instances 
	// ===========================================================
	private BallInfeed _ballInfeed = BallInfeed.getInstance();
	private Chassis _chassis = Chassis.getInstance();
	private Climber _climber = Climber.getInstance();
	private GearHandler _gearHandler = GearHandler.getInstance();
	private Shooter _shooter = Shooter.getInstance();
	
	private DashboardInputs _dashboardInputs = DashboardInputs.getInstance();
	private DriversStation _driversStation = DriversStation.getInstance();
	
	private AutonExecuter _autoModeExecuter = null;
	
	// sensors
	private NavXGyro _navX = NavXGyro.getInstance();
	private SwitchableCameraServer _switchableCameraServer;
	private RoboRealmClient _roboRealmClient = RoboRealmClient.getInstance();
	
	// Wrapper around data logging (will be null if logging is not enabled)
	private DataLogger _dataLogger;
	
	// ===========================================================
	//   Define class level instance variables for Robot State
	// ===========================================================
	private TELEOP_MODE _teleopMode;
	private AUTON_MODE _autonMode;
	private ALLIANCE_COLOR _allianceColor;
	
	// ===========================================================
	//   Define class level instance variables for Robot Controllers
	// ===========================================================
	private AutoShootController _autoShootController;
	private ChassisAutoAimController _chassisAutoAimGyro;
	private HangGearController _hangGearController = HangGearController.getInstance();
	private TrajectoryDriveController _trajController = TrajectoryDriveController.getInstance();
	
	// ===========================================================
	//   Define class level instance variables for Robot Auton Routines 
	// ===========================================================
	CrossBaseLine _crossBaseLineAuton;
	DoNothing _doNothingAuton;
	HangBoilerGear _hangBoilerGearAuton;
	HangBoilerGearAndShoot _hangBoilerGearAndShootAuton;
	HangCenterGear _hangCenterGearAuton;
	HangCenterGearAndShoot _hangCenterGearAndShootAuton;
	HangRetrievalGear _hangRetrievalGear;
	HitHopper _hitHopper;
	TurnAndShoot _turnAndShoot;
	//TwoGear _twoGearAuton;
	
	// ===========================================================
	//   Define class level working variables
	// ===========================================================
	String _buildMsg = "?";
	ShooterTable _shooterTable = ShooterTable.getInstance();
	String _fmsDebugMsg = "?";
 	long _lastDashboardWriteTimeMSec;
 	long _lastScanEndTimeInMSec;
 	MovingAverage _scanTimeSamples;
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed 1x at robot startup																		ROBOT INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void robotInit() {
        //===================
    	// write jar (build) date & time to the dashboard
        //===================
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
		
		// sensors follow
		//_lidar = new Lidar(SerialPort.Port.kMXP);		// TODO: Re-enable?
		
		_switchableCameraServer = new SwitchableCameraServer("cam0");			//safe
												
												//RobotMap.PCM_CAN_BUS_ADDR,
												//RobotMap.GEAR_LED_RING_PCM_PORT,
												//RobotMap.BOILER_LED_RING_PCM_PORT);
		
		// telop Controller follow
		_chassisAutoAimGyro = new ChassisAutoAimController();
		_autoShootController = new AutoShootController(_chassisAutoAimGyro, _roboRealmClient, _shooter, _shooterTable);
				
		// debug info for FMS Alliance sensing
		boolean isFMSAttached = _dashboardInputs.getIsFMSAttached();
		//_fmsDebugMsg = "Is FMS Attached: [" + isFMSAttached + "] Alliance: [" + allianceColor + "]";
		DriverStation.reportWarning(">>>>> " + _fmsDebugMsg + " <<<<<<", false);
		
		// create class to hold Scan Times moving Average samples
		_scanTimeSamples = new MovingAverage(100);  // 2 sec * 1000mSec/Sec / 20mSec/Scan
		SmartDashboard.putString("Scan Time (2 sec roll avg)", "0.0 mSec");
		
		//Update Dashboard Fields (push all fields to dashboard)
		OutputAllToSmartDashboard();
	}
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// called each time the robot enters disabled mode from either telop or auton mode							DISABLED PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void disabledPeriodic() {
		// if logging was enabled, make sure we close the file
    	if(_dataLogger != null) {
	    	_dataLogger.close();
	    	_dataLogger = null;
    	}
    	
    	if(_autoModeExecuter != null) {
    		_autoModeExecuter.stop();
    		_autoModeExecuter = null;
    	}
		
		if(_roboRealmClient != null) {
			_roboRealmClient.TurnAllVisionLEDsOff();
		}
	}
		
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// code executed 1x when entering AUTON Mode																AUTONOMOUS INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void autonomousInit() {
		// =====================================
    	// Step 1: pre-state cleanup
		// =====================================
		
		// stop gear
    	_gearHandler.stop();
    	_gearHandler.ZeroGearTiltAxisInit();
    	
    	_hangGearController.setIsChassisControlEnabled(false);
    	
    	_trajController.startTrajectoryController();
    	
    	// =====================================
		// Step 2: Read from Dashboard Choosers to select the Auton routine to run
    	// =====================================
    	if(_autoModeExecuter != null) {
    		_autoModeExecuter.stop();
    		_autoModeExecuter = null;
    	}
    	
    	_autoModeExecuter = new AutonExecuter();
    	_autoModeExecuter.setAutoMode(_dashboardInputs.getSelectedAuton());
    	_autoModeExecuter.start();
    	
		
    	// =====================================
    	// Step 2.3: Turn all vision LEDs offg
    	// =====================================
    	_roboRealmClient.TurnAllVisionLEDsOff();
    	
       	//=========================================================
    	//Step 2.3
    	//Set the Proper Cameras for this match
    	//==========================================================
    	/*_switchableCameraServer.setGearCameraName(_dashboardInputs.get_gearCam().get_cameraName());
    	_switchableCameraServer.setShooterCameraName(_dashboardInputs.get_shooterCam().get_cameraName());
    	_switchableCameraServer.setClimberCameraName(_dashboardInputs.get_climberCam().get_cameraName());
    	_switchableCameraServer.setDriverCameraName(_dashboardInputs.get_driverCam().get_cameraName());*/
    	
    	// =====================================
    	// Step 3: Optionally Configure Logging
    	// =====================================
    	_dataLogger = GeneralUtilities.setupLogging("auton");
    	
    	_lastDashboardWriteTimeMSec = new Date().getTime();
	}

	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in AUTON mode							AUTONOMOUS PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void autonomousPeriodic() {
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
      	
    	// =====================================
    	// Step 3: Optionally Log Data
    	// =====================================
		WriteLogData();
		
		OutputAllToSmartDashboard();
	}

	// --------------------------------------------------------------------------------------------------------------------------------------------
	// code executed 1x when entering TELOP Mode																TELOP INIT
	// --------------------------------------------------------------------------------------------------------------------------------------------

	@Override
	public void teleopInit() {
    	// =====================================
    	// Step 1: Setup Robot Defaults
    	// =====================================
		// #### Chassis ####
    	_chassis.stop(); 								// Stop Motors
    	_chassis.zeroSensors(); 						// Zero drive encoders
    	
    	// #### Climber ####
    	_climber.stop();
    	
    	// #### GearHandler ####
    	_gearHandler.stop();
    	if(!_gearHandler.hasTiltAxisBeenZeroed()) { 
    		_gearHandler.ZeroGearTiltAxisInit(); 
    	} else {
    		_gearHandler.MoveGearToScorePosition();
    	}
    	
    	_hangGearController.setIsChassisControlEnabled(true);

    	// #### Shooter ####
    	_shooter.stop();
    	_shooter.MoveActuatorToDefaultPosition();
    	_shooter.ResetHopperCarousel();
    	
    	// #### Ball Infeed ####
    	_ballInfeed.stop();
    	
    	_navX.zeroYaw();
    	
    	// #### Cameras ####
    	// set to default camera
    	//_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
    	
    	// #### Telop Controller ####
    	_teleopMode = TELEOP_MODE.STANDARD;	// default to std mode
    	
    	_roboRealmClient.ChangeToCamera(VISION_CAMERAS.BOILER);
    	
    	// #### Lidar starts doing ####
    	//if(_lidar != null)	{ _lidar.start(); }	//TODO: resolve timeout
    	
    	//=========================================================
    	//Step 2
    	//Set the Proper Cameras for this match
    	//==========================================================
    	/*_switchableCameraServer.setGearCameraName(_dashboardInputs.get_gearCam().get_cameraName());
    	_switchableCameraServer.setShooterCameraName(_dashboardInputs.get_shooterCam().get_cameraName());
    	_switchableCameraServer.setClimberCameraName(_dashboardInputs.get_climberCam().get_cameraName());
    	_switchableCameraServer.setDriverCameraName(_dashboardInputs.get_driverCam().get_cameraName());*/
    	
    	// =====================================
    	// Step 2.2: Turn all vision LEDs offg
    	// =====================================
    	_roboRealmClient.TurnAllVisionLEDsOff();
    	
    	// =====================================
    	// Step 3: Configure Logging (if USB Memory Stick is present)
    	// =====================================    	
    	_dataLogger = GeneralUtilities.setupLogging("telop");
    	
    	_lastDashboardWriteTimeMSec = new Date().getTime();
	}
	
	// --------------------------------------------------------------------------------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in TELOP Mode							TELOP PERIODIC
	// --------------------------------------------------------------------------------------------------------------------------------------------
	@Override
	public void teleopPeriodic() {
    	// =====================================
    	// Step 0: Get the DASHBOARD Input values for the current scan cycle
    	// =====================================
    	_driversStation.ReadCurrentScanCycleValues();
    	
    	// =====================================
    	// Step 1: execute different steps based on current "telop mode"
    	// =====================================
    	switch (_teleopMode) {
    		case STANDARD:	  
    			//DriverStation.reportError(Double.toString(_navX.getYaw()), false);
    			//===========================================================================
    			//Switchable Cameras
    			//=======================================================================			
    			if(_driversStation.getIsOperator_SwapCamera_BtnJustPressed()) {
    				_switchableCameraServer.ChgToNextCamera();
    			}
    			else if (_driversStation.getIsEngineering_SwapCamera_BtnJustPressed()) {
    				_switchableCameraServer.ChgToNextCamera();
    			}
				//=====================
		    	// Chassis Gear Shift
				//=====================
		    	if(_driversStation.getIsDriver_ShiftDrivingGear_BtnJustPressed()) {
		    		_chassis.ToggleShiftGear();
		    	}

		    	//=====================
		    	// Chassis Throttle Cmd
				//=====================
		    	if ((Math.abs(_driversStation.getDriver_ChassisThrottle_JoystickCmd()) > 0.0) 
		    			|| (Math.abs(_driversStation.getDriver_ChassisTurn_JoystickCmd()) > 0.0)) {
		    		// std drive
			    	_chassis.ArcadeDrive((_driversStation.getDriver_ChassisThrottle_JoystickCmd() * -1.0), 	// added -1 gear is front
											_driversStation.getDriver_ChassisTurn_JoystickCmd());
		    	} 
		    	else if ((Math.abs(_driversStation.getDriver_SpinChassisLeft_JoystickCmd()) > 0.0)
		    				&& (Math.abs(_driversStation.getDriver_SpinChassisRight_JoystickCmd()) == 0.0)) {
		    		// spin left
		    		_chassis.ArcadeDrive(0.0, _driversStation.getDriver_SpinChassisLeft_JoystickCmd() * 0.75 * -1.0);
		    	} 
		    	else if ((Math.abs(_driversStation.getDriver_SpinChassisRight_JoystickCmd()) > 0.0) 
		    				&& (Math.abs(_driversStation.getDriver_SpinChassisLeft_JoystickCmd()) == 0.0)) {
		    		// spin right
		    		_chassis.ArcadeDrive(0.0, _driversStation.getDriver_SpinChassisRight_JoystickCmd() * 0.75);
		    	}
		    	else if ((Math.abs(_driversStation.getEngineering_SpinChassisLeft_JoystickCmd()) > 0.0)
	    				&& (Math.abs(_driversStation.getEngineering_SpinChassisRight_JoystickCmd()) == 0.0)) {
		    		// spin left
		    		_chassis.ArcadeDrive(0.0, _driversStation.getEngineering_SpinChassisLeft_JoystickCmd() * 0.75 * -1.0);
		    	} 
		    	else if ((Math.abs(_driversStation.getEngineering_SpinChassisRight_JoystickCmd()) > 0.0) 
		    				&& (Math.abs(_driversStation.getEngineering_SpinChassisLeft_JoystickCmd()) == 0.0)) {
		    		// spin right
		    		_chassis.ArcadeDrive(0.0, _driversStation.getEngineering_SpinChassisRight_JoystickCmd() * 0.75);
		    	} 
		    	else if (_driversStation.getIsOperator_VisionAim_BtnPressed()
		    				|| _driversStation.getIsEngineering_VisionAim_BtnPressed()) {
		    		// Turn on auto aiming with vision (for boiler)
		    		_chassisAutoAimGyro.motionMagicMoveToTarget(_navX.getYaw() - (_roboRealmClient.get_Angle()/3.5));
		    	} else {
		    		// full stop
		    		// 23.Apr.2017 TomB removed comments
		    		// this avoids ERROR  1  Robot Drive... Output not updated often enough.  java.lang.Thread.run(Thread.java:745) error message
			    	_chassis.ArcadeDrive(0.0, 0.0);
		    	}
		    	
    			//============================================================================
    			// Fuel Infeed Cmd
    			//===========================================================================   			
    			_ballInfeed.InfeedFuelAndExtendSolenoid(_driversStation.getOperator_FuelInfeedOutfeed_JoystickCmd());
    			    			
    			//=====================
    			// Shooter Table
    			//=====================	
    			//if (_driversStation.getIsOperator_IndexShooterSettingsDown_BtnPressed()
    			//	&& _driversStation.getIsOperator_IndexShooterSettingsUp_BtnPressed())
    			if (_driversStation.getIsOperator_AutoDistance_BtnPressed())
    			{
    				_shooter.CalcAutomaticShooter(_roboRealmClient.get_DistanceToBoilerInches());
    			}
    			else if(_driversStation.getIsOperator_IndexShooterSettingsUp_BtnJustPressed()) {
    				_shooter.IndexShooterTableUp();
    			} 
    			else if (_driversStation.getIsOperator_IndexShooterSettingsDown_BtnJustPressed()) {
    				_shooter.IndexShooterTableDown();
    			}
    			
    			//=====================
    			// Shooter Slider (manual control)
    			//=====================		
    			if(_driversStation.getIsOperator_MoveShooterSliderUp_BtnJustPressed()) {
    				_shooter.MoveActuatorUp();
    			} 
    			else if (_driversStation.getIsOperator_MoveShooterSliderDown_BtnJustPressed()) {
    				_shooter.MoveActuatorDown();
    			}
    			else if(_driversStation.getIsEngineering_MoveShooterSliderUp_BtnJustPressed()) {
    				_shooter.MoveActuatorUp();
    			} 
    			else if (_driversStation.getIsEngineering_MoveShooterSliderDown_BtnJustPressed()) {
    				_shooter.MoveActuatorDown();
    			}
    			
    			//=====================
    			// Run Shooter Motors
    			//=====================
				// Stg 1 Bump Up / Down
    			if(_driversStation.getIsEngineering_BumpStg1RPMUp_BtnJustPressed()) {
    				_shooter.BumpStg1MtrRPMUp();
    			}
    			else if (_driversStation.getIsEngineering_BumpStg1RPMDown_BtnJustPressed()) {
    				_shooter.BumpStg1MtrRPMDown();
				}

    			// Stg 2 Bump Up / Down
    			if(_driversStation.getIsEngineering_BumpStg2RPMUp_BtnJustPressed()) {
    				_shooter.BumpStg2MtrRPMUp();
    			}
    			else if (_driversStation.getIsEngineering_BumpStg2RPMDown_BtnJustPressed()) {
    				_shooter.BumpStg2MtrRPMDown();
				}
    			    	
    			//=====================
    			// Toggle Shooter Motors
    			//=====================
    			if(_driversStation.getIsOperator_ToggleShooterMotors_BtnJustPressed()) {
    				_shooter.ToggleShooterMotors();
    			}
    			else if(_driversStation.getIsEngineering_ToggleShooterMotors_BtnJustPressed()) {
    				_shooter.ToggleShooterMotors();
    			}
    			else if (_shooter.get_isShooterMotorsReentrantRunning()) {
    				_shooter.ShooterMotorsReentrant();
    			}
    			 			
    			//=====================
    			// Shooter Feeder (Magic Carpet & High Roller) Motors controlled as a unit
    			//=====================
    			if(_driversStation.getIsOperator_RunShooterFeederInReverse_BtnPressed()){
    				_shooter.RunShooterFeederInReverse();
    			}
    			
    			else if(_driversStation.getOperator_FireBall_BtnPressed()
    					&& !_shooter.get_isShooterInfeedReentrantRunning()) {
    				// start motors on initial press
    				_shooter.ToggleRunShooterFeeder();
    				_shooter.ToggleHopperCarousel();
    			}
    			else if(_driversStation.getEngineering_FireBall_BtnPressed()
    					&& !_shooter.get_isShooterInfeedReentrantRunning()) {
    				// start motors on initial press
    				_shooter.ToggleRunShooterFeeder();
    				_shooter.ToggleHopperCarousel();
    			}
    			
    			else if(_driversStation.getOperator_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// keep calling if still pressed
    				_shooter.RunShooterFeederReentrant();
    				_shooter.RunHopperCarousel();
    			}
    			else if(_driversStation.getEngineering_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// keep calling if still pressed
    				_shooter.RunShooterFeederReentrant();
    				_shooter.RunHopperCarousel();
    			}
    			
    			else if(!_driversStation.getOperator_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// if it was running and is no longer pressed
    				_shooter.ToggleRunShooterFeeder();
    			}
    			else if(!_driversStation.getEngineering_FireBall_BtnPressed()
    					&& _shooter.get_isShooterInfeedReentrantRunning()) {
    				// if it was running and is no longer pressed
    				_shooter.ToggleRunShooterFeeder();
    			} else {
    				// we need to shut off the motors if they were running in reverse and the reverse button was released
    				_shooter.CleanupRunShooterFeederInReverse();
    			}
    			
    					
		    	//=====================
		    	// Gear Tilt Cmd
		    	//	Note: All of the Gear Handler sequences are interruptable except for Zero!
				//=====================
		      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
		      		// 1st priority is zeroing
		      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		    		_gearHandler.ZeroGearTiltAxisReentrant();
		    	}
		      	else if (_driversStation.getIsDriver_RezeroGearTilt_ButtonJustPressed()) {
		      		 //2nd priority is operator request to rezero
		      		_gearHandler.ZeroGearTiltAxisInit();	// zeroing will start on next scan
		      	}
		      	//else if (Math.abs(_driversStation.getOperator_GearTiltFeed_JoystickCmd()) > 0.0) {
		      		// 3rd priority is joystick control
		      		//_gearHandler.MoveTiltAxisVBus(_driversStation.getOperator_GearTiltFeed_JoystickCmd(), false);
		      	//}
		      	else if (_driversStation.getIsDriver_SendGearTiltToHome_BtnJustPressed()) {
		      		// 4th priority is Goto Home
		      		_gearHandler.MoveGearToHomePosition();
		      		//DriverStation.reportError("NavX: " + Double.toString(_navX.getYaw()), false);
		      		//DriverStation.reportError("Vision: " + Double.toString(_roboRealmClient.get_Angle()), false);
		      	}
		      	else if (_driversStation.getIsDriver_SendGearTiltToScore_BtnJustPressed()) {
		      		// 5th priority is Goto Score
		      		_gearHandler.MoveGearToScorePosition();
		      	} 
		      	else if (_driversStation.getIsDriver_SendGearTiltToFloor_BtnJustPressed()
		      				|| !_gearHandler.getIsLastTiltMoveToFloorCallComplete()) {
		      		// 6th priority is Goto Floor
		      		// 	Note: MoveToFloor will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		      		_gearHandler.MoveGearToFloorPositionReentrant();
		      	} 
		      	
		    	//=====================
		    	// Gear Infeed/Outfeed Cmd
				//=====================
		      	
		      	if (_driversStation.getIsDriver_InfeedGear_BtnPressed()) {
		      		_gearHandler.SpinInfeedWheelsVBus(1.0);
		      	}
		      	else if (_driversStation.getIsDriver_OutfeedGear_BtnPressed()) {
		      		_gearHandler.SpinInfeedWheelsVBus(-1.0);
		      	}
		      	else {
		      		_gearHandler.SpinInfeedWheelsVBus(0.0);
		      	} 
		      	
				//=====================
		    	// ====> Enter Gear Hang Mode
				//=====================
    			if(_driversStation.getIsDriver_StartGearScoreSequence_BtnJustPressed()) {
    				// make sure Gear Tilt has completed zeroing before entering this mode!
    				if(_gearHandler.hasTiltAxisBeenZeroed()) {
	    				_teleopMode = TELEOP_MODE.HANG_GEAR_SEQUENCE_MODE;
	    				_hangGearController.Initialize();
    				} else {
    					DriverStation.reportWarning("=!=!= Cannot chg to Hang Gear Seq, Tilt is NOT finished zeroing yet =!=!=", false);
    				}
    			}	
		      	
    	    	// =====================================
    	    	// ====> Enter Climb Mode
    	    	// =====================================
    	    	
    			// climb is now a manual mode for Pittsburgh
    			_climber.RunMotorUsingJoyStick(_driversStation.getOperator_ClimberSpeed_JoystickCmd());
    	    	
		      	break;	// end of _telopMode = STANDARD
      		
    		case HANG_GEAR_SEQUENCE_MODE:
    			
    			// in this teleop mode the driver & operator do not have control until
    			// the sequence completes or it times out
    			boolean isStillRunning = _hangGearController.ExecuteRentrant();
    			
    			// if not still running, switch back to std teleop mode
    			//	(ie: give control back to the driver & operator)
    			if(!isStillRunning) {
    				_teleopMode = TELEOP_MODE.STANDARD;
    			}
    			break;	// end of _telopMode = HANG_GEAR_SEQUENCE_MODE   			
    	}	// end of switch statement

    	// =====================================
    	// Step N: Finish up 
    	// =====================================
    	
    	// Refresh Dashboard
    	OutputAllToSmartDashboard();
    	
    	// Optionally Log Data
    	WriteLogData();
	}
	
	//--------------------------------------------------------------------------------------------------------------------------------------------------------
	//  Utility / Helper Methods Follow
	//--------------------------------------------------------------------------------------------------------------------------------------------------------
	
    // utility method that calls the outputToSmartDashboard method on all subsystems
    private void OutputAllToSmartDashboard() {
    	// limit spamming
    	
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
	    	if(_ballInfeed != null)				{ _ballInfeed.outputToSmartDashboard(); }
	    	
	    	if(_chassis != null) 				{ _chassis.outputToSmartDashboard(); }
	    	
	    	if(_climber != null)				{ _climber.outputToSmartDashboard(); }
	    	
	    	if(_driversStation != null)			{ _driversStation.OutputToSmartDashboard(); }
	    	
	    	if(_gearHandler != null)			{ _gearHandler.outputToSmartDashboard(); }
	    	
	    	if(_navX != null)					{ _navX.OutputToSmartDashboard(); }
	    	
	    	if(_shooter != null)				{ _shooter.outputToSmartDashboard(); }
	    	
	    	if(_switchableCameraServer != null) { _switchableCameraServer.OutputToSmartDashboard(); }
	    	
	    	if(_roboRealmClient != null) 		{ _roboRealmClient.OutputToSmartDashboard(); }
	    	
	    	//if(_trajController != null)			{ _trajController.OutputToSmartDashboard(); }
	    	
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	SmartDashboard.putString("FMS Debug Msg", _fmsDebugMsg);
	    	
	    	BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	DecimalFormat df = new DecimalFormat("####");
	    	SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
	    	
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
    }
         
    // this method optionally calls the UpdateLogData on each subsystem and then logs the data
    private void WriteLogData() {    	
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data
        	// 23.Apr.2017 TomB Commented most out to improve logging perf
	    	if(_chassis != null) 			{ _chassis.updateLogData(logData); }
	    	
	    	//if(_climber != null) 			{ _climber.updateLogData(logData); }
	    		
	    	//if(_driversStation != null) 	{ _driversStation.UpdateLogData(logData); }
	    	
	    	//if(_gearHandler != null) 		{ _gearHandler.updateLogData(logData); }
	    	
	    	//if(_ballInfeed != null) 		{ _ballInfeed.updateLogData(logData); }
	    	
	    	//if(_lidar != null)				{ _lidar.UpdateLogData(logData); }
	    	
	    	//if(_navX != null) 				{ _navX.UpdateLogData(logData); }
	    	
	    	if(_shooter != null)			{ _shooter.updateLogData(logData); }
	    	
	    	//if(_roboRealmClient != null) 	{ _roboRealmClient.UpdateLogData(logData); }
	    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
    }
}