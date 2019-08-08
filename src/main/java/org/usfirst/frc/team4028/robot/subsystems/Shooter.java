package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.utilities.ShooterTable;
import org.usfirst.frc.team4028.robot.utilities.ShooterTableEntry;

import java.util.Date;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the SHOOTER (& Blender) Subsystem
//=====> For Changes see Prat Bruns

//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		Patrick		2/16 8:47		Enabling Blender and Feeder Motors
//	1		Patrick		2/18 5:36		Code Review
//	2		Patrick		2/20 10:02		Code Review on Shooter Testing
//	3		Patrick		2/20 18:47		Updating Values Written to SmartDashboard
//	4		Patrick		2/22 12:34		Making toggle button for Blender/Feeder
//	5		Patrick		3/1	 5:57		Toggle Blender Speed
//	6		Patrick		3/4	 11:06		Updating to Log %Voltage
//	7		Patrick		3/4	 1:31		Changing PID Values
//  8		TomB		9.Mar.2017		Refactor for new infeed 3 motor combo
//-------------------------------------------------------------
public class Shooter extends Subsystem{
	public static Shooter _instance = new Shooter();
	
	public static Shooter getInstance() {
		return _instance;
	}
	
	// =====================================================================
	// 5 DC Motors
	//		1 Talon w/ Encoder, 	PID V Mode		2nd Stage
	//		1 Talon w/ Encoder, 	PID V Mode		1st Stage
	//		1 Talon w/o Encoder,	% VBus Mode		Magic Carpet Motor
	//		1 Talon w/o Encoder,	% VBus Mode		High Speed Infeed Lane
	//		1 Talon w/o Encoder,	% VBus Mode		High Roller
	//
	// 1 Servo
	// 		I Linear Actuator		PWM				Slider
	// =====================================================================
	
	// define class level variables for Robot objects`
	private TalonSRX _firstStgMtr;
	private TalonSRX _secondStgMtr; 
	private TalonSRX _magicCarpetMtr;
	private TalonSRX _highSpeedInfeedLaneMtr;
	private TalonSRX _highRollerMtr;
	
	private Servo _linearActuator;
	private Servo _hopperCarousel;
	private Servo _hopperServo;
	
	// define class level working variables
	private ShooterTable _shooterTable;
	private ShooterTableEntry _currentShooterTableEntry;
	private ShooterTableEntry _lastShooterTableEntry;
	
	private double _stg1MtrTargetRPM;
	private double _stg2MtrTargetRPM;
	private boolean _isStg1MtrTargetRPMBumpingUp;
	private boolean _isStg2MtrTargetRPMBumpingUp;
	
	private double _magicCarpetMtrTargetVBus;	
	private double _highSpeedInfeedLaneMtrTargetVBus;
	private double _highRollerMtrTargetVBus;

	private double _currentSliderPosition;
	private boolean _isShooterMotorsReentrantRunning = false;
	private boolean _isShooterInfeedReentrantRunning = false;
	private long _shooterInfeedReentrantRunningMsec;
	
	private long _hopperCarouselReentrantRunningMsec;
	private long _lastDebugWriteTimeMSec;
	
	//==============================================================================================
	//define class level PID constants					// new	// clev
	
	// Post Worlds
	/*
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.027; //0.033; //
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.5; //0.45; // 0.4; //0.35; //0.30; // 0.25;// 0.325;    
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 1.5; //5.0; 	//2.0; //4.0; //7.5; //5.0; //

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.028; //0.03; //
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.5; // 0.175; 	//0.3; // .250; //0.175; //
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 3.0; //6.0; 	//3.0; //6.0; // 
	*/
	
	// Cleveland	
	
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.033; //
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.325;         // 0.3125; // 0.275; //0.225; //0.325; //
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 5.0; 	//2.0; //4.0; //7.5; //5.0; //

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.03; //
	private static final double SECOND_STAGE_MTG_P_GAIN =  0.175; 	//0.3; // .250; //0.175; //
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 6.0; 	//3.0; //6.0; // 
	

	//==============================================================================================
	
	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.80; //0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.35; //0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.01;
	private static final double INITIAL_POSITION_ACTUATOR = 0.65;
	
	//define class level Shooter Motor Constants
	private static final double MAX_SHOOTER_RPM = -4400;
	private static final double MIN_SHOOTER_RPM = -2500;
	private static final double SHOOTER_BUMP_RPM = 25;
	private static final double FIRST_STAGE_MTR_DEFAULT_RPM = -3500;
	private static final double SECOND_STAGE_MTR_DEFAULT_RPM = -3200;
	
	// define class level Ball Infeed Motor Constants
	private static final double MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND = -0.7;
	private static final double HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND = 0.7;
	private static final double HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND = -0.85; //-9.0; //-0.85;

	private static final double SHOOTER_WHEEL_WITHIN_SPEED_PERCENT_THRESHHOLD = 3.0;  // was 2.0
	//============================================================================================
	// CONSTRUCTORS FOLLOW
	//============================================================================================
	private Shooter() {
		// First Stage Motor
		_firstStgMtr = new TalonSRX(RobotMap.SHOOTER_STG1_CAN_BUS_ADDR);
		//_firstStgMtr.changeControlMode(ControlMode.Velocity);	// open loop throttle
		_firstStgMtr.setNeutralMode(NeutralMode.Coast);					// default to brake mode DISABLED
    	_firstStgMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // set encoder to be feedback device
    	_firstStgMtr.setSensorPhase(true);  							// do not invert encoder feedback
		
		_firstStgMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_firstStgMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        // set the peak and nominal outputs, 12V means full 

		_firstStgMtr.configNominalOutputForward(+0.0f, 0);
		_firstStgMtr.configNominalOutputReverse(-0.0f, 0);
		_firstStgMtr.configPeakOutputForward(0.0f, 0);
		_firstStgMtr.configPeakOutputReverse(-12.0f, 0); 

    	
		// set closed loop gains in slot0 
		_firstStgMtr.selectProfileSlot(0, 0);
		_firstStgMtr.config_kF(0, FIRST_STAGE_MTG_FF_GAIN); 
		_firstStgMtr.config_kP(0, FIRST_STAGE_MTG_P_GAIN); 
		_firstStgMtr.config_kI(0, FIRST_STAGE_MTG_I_GAIN); 
		_firstStgMtr.config_kD(0,FIRST_STAGE_MTG_D_GAIN);
		_firstStgMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
		//_firstStgMtr.SetVelocityMeasurementWindow(windowSize);
		
		// Second Stage Motor
		_secondStgMtr = new TalonSRX(RobotMap.SHOOTER_STG2_CAN_BUS_ADDR);
		//_secondStgMtr.changeControlMode(TalonSRX.TalonControlMode.Speed);	// open loop throttle
		_secondStgMtr.setNeutralMode(NeutralMode.Coast);							// default to brake mode DISABLED
    	_secondStgMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // set
																														// encoder
																														// to
																														// be
																														// feedback
																														// device
    	_secondStgMtr.setSensorPhase(true);					// do not invert encoder feedback
		
		_secondStgMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_secondStgMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    	//_secondStageMtr.reverseOutput(true);
        // set the peak and nominal outputs, 12V means full
		_secondStgMtr.configNominalOutputForward(+0.0f, 0);
		_secondStgMtr.configNominalOutputReverse(-0.0f, 0);
		_secondStgMtr.configPeakOutputForward(0.0f, 0);
		_secondStgMtr.configPeakOutputReverse(-12.0f, 0);
		// set closed loop gains in slot0
		_secondStgMtr.selectProfileSlot(0, 0);
		_secondStgMtr.config_kF(0, SECOND_STAGE_MTG_FF_GAIN); 
		_secondStgMtr.config_kP(0, SECOND_STAGE_MTG_P_GAIN); 
		_secondStgMtr.config_kI(0, SECOND_STAGE_MTG_I_GAIN); 
		_secondStgMtr.config_kD(0, SECOND_STAGE_MTG_D_GAIN);
				
		// Magic Carpet Motor
		_magicCarpetMtr = new TalonSRX(RobotMap.MAGIC_CARPET_CAN_BUS_ADDR);
		//_magicCarpetMtr.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		_magicCarpetMtr.setNeutralMode(NeutralMode.Coast);
		
		_secondStgMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_secondStgMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
		
		// High Speed Infeed Lane Motor
		_highSpeedInfeedLaneMtr = new TalonSRX(RobotMap.HIGH_SPEED_INFEED_LANE_CAN_BUS_ADDR);
		//_highSpeedInfeedLaneMtr.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		
		//_highSpeedInfeedLaneMtr.changeControlMode(TalonSRX.TalonControlMode.Voltage);
		//_highSpeedInfeedLaneMtr.setVoltageCompensationRampRate(24.0);
		_highSpeedInfeedLaneMtr.setNeutralMode(NeutralMode.Coast);
		
		_highSpeedInfeedLaneMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_highSpeedInfeedLaneMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
		
		// High Roller Motor
		_highRollerMtr = new TalonSRX(RobotMap.HIGH_ROLLER_CAN_BUS_ADDR);
		//_highRollerMtr.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
		_highSpeedInfeedLaneMtr.setNeutralMode(NeutralMode.Coast);
		
		_highRollerMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_highRollerMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
		
		// Slider
		_linearActuator = new Servo(RobotMap.SHOOTER_SLIDER_PWM_PORT);
		
		// Hopper Carousel
		_hopperCarousel = new Servo(8);
		_hopperServo = new Servo(7);
		
		// Default Feeder Subsystem working variables
		_magicCarpetMtrTargetVBus = 0;
		_highSpeedInfeedLaneMtrTargetVBus = 0;
		_highRollerMtrTargetVBus = 0;
		
		// default to bumping rmp up on both shooter motors
		_isStg1MtrTargetRPMBumpingUp = true;
		_isStg2MtrTargetRPMBumpingUp = true;
		
		// setup the shooter table
		_shooterTable = ShooterTable.getInstance();
		_currentShooterTableEntry = _shooterTable.getCurrentEntry();
	}
	
	//============================================================================================
	// METHODS FOLLOW
	//============================================================================================
	
	public void FullShooterStop() {
		RunStg1(0);
		RunStg2(0);
		RunHighSpeedInfeedLane(0.0);
		RunMagicCarpet(0.0);
		RunHighRoller(0.0);
		
		_isShooterMotorsReentrantRunning = false;
	}
	
	public void FullShooterFeederStop() 
	{
		RunHighSpeedInfeedLane(0.0);
		RunMagicCarpet(0.0);
		RunHighRoller(0.0);

		_isShooterInfeedReentrantRunning = false;
	}
	
	public void ToggleShooterMotors() {
		// base on Stg 2 since since it starts first
		if (_stg2MtrTargetRPM == 0.0) {
			ShooterMotorsReentrant();
		} else {
			stop();
		}
	}
	
	// This method is provided for auton mode shooting
	// keep calling this method until it returns false, that indicates motors are upto speed
	public boolean ShooterMotorsReentrant(ShooterTableEntry shooterTableEntry) {
		//_currentShooterTableEntry = shooterTableEntry;
		//_lastShooterTableEntry = shooterTableEntry;
		
		ShooterMotorsReentrant();
		// we use a 2% error threshhold
		if((Math.abs(getStg2RPMErrorPercent()) <= SHOOTER_WHEEL_WITHIN_SPEED_PERCENT_THRESHHOLD)			
				&& (Math.abs(getStg1RPMErrorPercent()) <= SHOOTER_WHEEL_WITHIN_SPEED_PERCENT_THRESHHOLD)) {
			return true;
		} else {
			return false;
		}
	}
	
	// this is the normal method used during teleop
	public boolean ShooterMotorsReentrant() {
		if(Math.abs(_stg2MtrTargetRPM) == 0) {
			RunStg2(_currentShooterTableEntry.Stg2MotorRPM);
			MoveActuatorToPosition(_currentShooterTableEntry.SliderPosition);
			
			//BumpStg2MtrRPMUp();
			_isShooterMotorsReentrantRunning = true;
		}
		else if(Math.abs(getStg2RPMErrorPercent()) > 7.5 ) {
			// allow time to spinup
			_isShooterMotorsReentrantRunning = true;
		}
		else if (Math.abs(_stg1MtrTargetRPM) == 0) {
			RunStg1(_currentShooterTableEntry.Stg1MotorRPM);
			//BumpStg1MtrRPMUp();
			_isShooterMotorsReentrantRunning = true;
		}
		else if(Math.abs(getStg1RPMErrorPercent()) > 5.0 ) {
			// allow time to spinup
			_isShooterMotorsReentrantRunning = true;
		} else {
			// dynamically adjust speeds if current table entry is changed
			if(_lastShooterTableEntry.Index != _currentShooterTableEntry.Index) {
				RunStg2(_currentShooterTableEntry.Stg2MotorRPM);
				RunStg1(_currentShooterTableEntry.Stg1MotorRPM);
				// allow user to override slider if we have not chg to a new index in the shooter table
				MoveActuatorToPosition(_currentShooterTableEntry.SliderPosition);
			}
		}
		
		// cache last so we can tell if we changed
		_lastShooterTableEntry = _currentShooterTableEntry;
		
		return _isShooterMotorsReentrantRunning;
	}
	
	//============================================================================================
	// Shooter Motors
	//============================================================================================

	public void RunStg1(double targetRPM) {
		_stg1MtrTargetRPM = targetRPM;
		_firstStgMtr.set(ControlMode.Velocity, _stg1MtrTargetRPM);
		DriverStation.reportWarning("Stage 1 Target RPM = " + targetRPM, false);
	}
	
	public void RunStg2(double targetRPM) {
		_stg2MtrTargetRPM = targetRPM;
		_secondStgMtr.set(ControlMode.Velocity, _stg2MtrTargetRPM);
		DriverStation.reportWarning("Stage 2 Target RPM = " + targetRPM, false);
	}
	
	//============================================================================================
	// Set Up Shooter Testing
	//============================================================================================
	
	public void IndexShooterTableUp() {
		_currentShooterTableEntry = _shooterTable.getNextEntry();
	}
	
	public void IndexShooterTableDown() {
		_currentShooterTableEntry = _shooterTable.getPreviousEntry();
	}
	
	public void BumpStg1MtrRPMUp() {
		// only bump if not already at max
		if(Math.abs(_stg1MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM)) {	
			//if(Math.abs(_stg1MtrTargetRPM) >  0)
			//{
				// if already turning, just bump
			//_stg1MtrTargetRPM = _stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
			RunStg1(_stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
			//}
			//else
			//{
				// if currently stopped, set to default speed
			//	RunStg1(FIRST_STAGE_MTR_DEFAULT_RPM);
			//}
		} else {
			DriverStation.reportWarning("Stg 1 Mtr Already at MAX ", false);
			_isStg1MtrTargetRPMBumpingUp = false;
		}
	}
	public void CalcAutomaticShooter (double distanceInInches)
	{
		// full calc from formulas
		//_currentShooterTableEntry = _shooterTable.CalcShooterValues(distanceInInches);
		
		// linear interpolation between shooter table values
		_currentShooterTableEntry = _shooterTable.CalcShooterValues2(distanceInInches);
	}
	
	public void BumpStg1MtrRPMDown() {
		// only bump if not already at min
		if(Math.abs(_stg1MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM)) {
			//_stg1MtrTargetRPM = _stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
			RunStg1(_stg1MtrTargetRPM += SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 1 Mtr Already at MIN ", false);
			_isStg1MtrTargetRPMBumpingUp = true;
		}
	}
	
	public void BumpStg2MtrRPMUp() {
		// only bump if not already at max
		if(Math.abs(_stg2MtrTargetRPM) <= Math.abs(MAX_SHOOTER_RPM)) {	
			//if(Math.abs(_stg2MtrTargetRPM) >  0)
			//{
				//_stg2MtrTargetRPM = _stg2MtrTargetRPM -= SHOOTER_BUMP_RPM;
				// if already turning, just bump
			RunStg2(_stg2MtrTargetRPM -= SHOOTER_BUMP_RPM);
				//DriverStation.reportWarning("Bumping Up Stage 2", false);
			//}
			//else
			//{
				// if currently stopped, set to default speed
			//	RunStg2(SECOND_STAGE_MTR_DEFAULT_RPM);
			//}
		} else {
			DriverStation.reportWarning("Stg 2 Mtr Already at MAX ", false);
			_isStg2MtrTargetRPMBumpingUp = false;
		}
	}

	public void BumpStg2MtrRPMDown() {
		// only bump if not already at min
		if(Math.abs(_stg2MtrTargetRPM) > Math.abs(MIN_SHOOTER_RPM)) {
			//_stg2MtrTargetRPM = _stg2MtrTargetRPM += SHOOTER_BUMP_RPM;
			RunStg2(_stg2MtrTargetRPM += SHOOTER_BUMP_RPM);
		} else {
			DriverStation.reportWarning("Stg 2 Mtr Already at MIN ", false);
			_isStg2MtrTargetRPMBumpingUp = true;
		}
	}
	
	/*
	public void ControlHighSpeedLane()
	{
		if(_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0) 
				&& !_isShooterInfeedReentrantRunning)
		{
			// Priority 1: run in reverse if other infeed motors are runing in reverse
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND * -1.0);
		} 
		else if(((Math.abs(_stg1MtrTargetRPM) > 0) && (Math.abs(getStg1RPMErrorPercent()) < 5))
				&& ((Math.abs(_stg2MtrTargetRPM) > 0) && (Math.abs(getStg2RPMErrorPercent()) < 5)))
		{
			// Priority 2: run High Speed Lane if both Shooter Motors are within 5% of non-zero cmd
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
		} else
		{
			// turn off
			RunHighSpeedInfeedLane(0.0);
		}
	}
	*/
	
	private void RunHighSpeedInfeedLane(double percentVbusCommand) {
		_highSpeedInfeedLaneMtrTargetVBus = percentVbusCommand;
		_highSpeedInfeedLaneMtr.set(ControlMode.PercentOutput, _highSpeedInfeedLaneMtrTargetVBus);
	}
	
	public void ToggleHighSpeedInfeedLane() {
		if (_highSpeedInfeedLaneMtrTargetVBus == 0.0) {
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
		} else {
			RunHighSpeedInfeedLane(0.0);
		}
	}
	
	//============================================================================================
	// Run Magin Carpet / High Roller Motors / Hopper Carousel
	//============================================================================================
	public void ResetHopperCarousel() {
		_hopperCarousel.setPosition(1.0);
		_hopperServo.setPosition(1.0);
	}
	
	public void ToggleHopperCarousel() {
		_hopperCarouselReentrantRunningMsec = System.currentTimeMillis();
	}
	
	public void RunHopperCarousel() {
		if ((System.currentTimeMillis() - _hopperCarouselReentrantRunningMsec) < 750) {
			_hopperCarousel.setPosition(0.0);
			_hopperServo.setPosition(0.0);
		} else if ((System.currentTimeMillis() - _hopperCarouselReentrantRunningMsec) < 1500) {
			_hopperCarousel.setPosition(1.0);
			_hopperServo.setPosition(1.0);
		} else {
			_hopperCarouselReentrantRunningMsec = System.currentTimeMillis();
		}
		
	}
	
	public void ToggleRunShooterFeeder() {
		// if current cmd is 0 or - if running in reverse, then start
		//if(_magicCarpetMtrTargetVBus != MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND)
		if(!_isShooterInfeedReentrantRunning
				|| (_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0))) {
			//RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
			//RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND);
			//RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND);
			ToggleHopperCarousel();
			
			_isShooterInfeedReentrantRunning = true;
			_shooterInfeedReentrantRunningMsec = System.currentTimeMillis();
		} else {
			FullShooterFeederStop();
			
			_isShooterInfeedReentrantRunning = false;
		}
	}
	
	public void RunShooterFeederReentrant() {
		RunHopperCarousel();
		if((System.currentTimeMillis() - _shooterInfeedReentrantRunningMsec) < 100)	{		// 800 ; 500
			// 100 mSec fwd
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
		}
		else if((System.currentTimeMillis() - _shooterInfeedReentrantRunningMsec) < 1800) {	// 800 ; 500
			// 1800 mSec fwd
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
			RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND);
			RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND);
		}
		else if((System.currentTimeMillis() - _shooterInfeedReentrantRunningMsec) < 1840) {	// 40; 20; 40
			// 40 mSec pause
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
			RunMagicCarpet(0.0);
			RunHighRoller(0.0);
		}
		else if((System.currentTimeMillis() - _shooterInfeedReentrantRunningMsec) < 1880) {	// 40; 40
			// 40 mSec rev
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
			RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0);
			RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND * -1.0);
		}
		else if((System.currentTimeMillis() - _shooterInfeedReentrantRunningMsec) < 1920) {	// 40; 20; 40
			// 40 mSec pause
			RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND);
			RunMagicCarpet(0.0);
			RunHighRoller(0.0);
		} else {
			_shooterInfeedReentrantRunningMsec = System.currentTimeMillis();
		}
	}
	
	public void RunShooterFeederInReverse() {
		RunHighSpeedInfeedLane(HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND * -1.0);
		RunMagicCarpet(MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0);
		RunHighRoller(HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND * -1.0);
	}
	
	public void CleanupRunShooterFeederInReverse() {
		if (_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0)) {
			FullShooterFeederStop();
		}
	}
	
	private void RunMagicCarpet(double percentVbusCommand) {
		_magicCarpetMtrTargetVBus = percentVbusCommand;
		_magicCarpetMtr.set(ControlMode.Velocity, _magicCarpetMtrTargetVBus);
	}
	
	private void RunHighRoller(double percentVbusCommand) {
		_highRollerMtrTargetVBus = percentVbusCommand;
		_highRollerMtr.set(ControlMode.PercentOutput, percentVbusCommand);
	}
	
	//============================================================================================
	// Linear Actuator
	//============================================================================================
	
	public void MoveActuatorToDefaultPosition() {
		_currentSliderPosition = INITIAL_POSITION_ACTUATOR;
		_linearActuator.setPosition(_currentSliderPosition);
		
		DriverStation.reportWarning("Actuator Configured to " + _currentSliderPosition, false);
	} 
	
	public void MoveActuatorToPosition(double sliderPosition) {
		_currentSliderPosition = sliderPosition;
		_linearActuator.setPosition(_currentSliderPosition);
		
		DriverStation.reportWarning("Actuator Moved to " + _currentSliderPosition, false);
	} 
	
	public void MoveActuatorUp() {
		// are we below the max?
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR) {
			// increment the target position
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition > MAX_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MAX_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Up To: " + _currentSliderPosition, false);
		} else {
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MAXIMUM Position", false);
		}
	}
	
	public void MoveActuatorDown() {
		// are we above the min?
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR) {
			// decrement the target position
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			
			// protect fwd limit
			if(_currentSliderPosition < MIN_THRESHOLD_ACTUATOR) {
				_currentSliderPosition = MIN_THRESHOLD_ACTUATOR;
			}
			
			//rounds to 3 Decimal Places
			_currentSliderPosition = GeneralUtilities.RoundDouble(_currentSliderPosition, 3); 
			
			// actually move the slider
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Down To: " + _currentSliderPosition, false);
		} else {
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MINIMUM Position", false);
		}
	}
	
	//============================================================================================
	// PROPERTY ACCESSORS FOLLOW
	//============================================================================================
	
	// ---- Stage 1 ----------------------
	private double getStg1ActualRPM() {
		return _firstStgMtr.getMotorOutputPercent();
	}
	
	private double getStg1RPMErrorPercent() {
		if(Math.abs(_stg1MtrTargetRPM) > 0 ) {		
			return ((_stg1MtrTargetRPM - getStg1ActualRPM()) / _stg1MtrTargetRPM) * 100.0 * -1.0;
		} else {
			return 0.0;
		}
	}
	
	private double getStg1CurrentPercentVBus() {
		double currentOutputVoltage = _firstStgMtr.getMotorOutputVoltage();
		double currentBusVoltage = _firstStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}
	
	// ---- Stage 2 ----------------------
	private double getStg2ActualRPM() {
		return _secondStgMtr.getMotorOutputPercent();
	}
	
	private double getStg2RPMErrorPercent() {
		if(Math.abs(_stg2MtrTargetRPM) > 0 ) {		
			return ((_stg2MtrTargetRPM - getStg2ActualRPM()) / _stg2MtrTargetRPM) * 100.0 * -1.0;
		} else {
			return 0.0;
		}
	}
	
	private double getStg2CurrentPercentVBus() {
		double currentOutputVoltage = _secondStgMtr.getMotorOutputVoltage();
		double currentBusVoltage = _secondStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return GeneralUtilities.RoundDouble(currentActualSpeed, 2);
	}
	
	public boolean get_isShooterMotorsReentrantRunning() {
		return _isShooterMotorsReentrantRunning;
	}
	
	public boolean get_isShooterInfeedReentrantRunning() {
		return _isShooterInfeedReentrantRunning;
	}

	@Override
	public void stop() {
		FullShooterStop();
		FullShooterFeederStop();
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void outputToSmartDashboard() {
		//%s - insert a string
		//%d - insert a signed integer (decimal)
		//%f - insert a real number, standard notation
		
		// working varibles
		String stg1MtrData = "?";
		String stg2MtrData = "?";
		String shooterFeederMtrsData = "?";
		String actuatorData = "?";
		String currentShooterTableValues = "?";
		
		//Display Current Shooter Motor 1 & 2  | Target | Actual RPM | Error | %Out
		stg1MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.0f%%]", 
												-1*_stg1MtrTargetRPM, 
												-1*getStg1ActualRPM(), 
												getStg1RPMErrorPercent(),
												getStg1CurrentPercentVBus() * 100);
		
		stg2MtrData = String.format("[%.0f RPM] %.0f RPM (%.2f%%) [%.0f%%]", 
												-1*_stg2MtrTargetRPM, 
												-1*getStg2ActualRPM(), 
												getStg2RPMErrorPercent(),
												getStg2CurrentPercentVBus() * 100);
		
		SmartDashboard.putString("Stg 1 RPM Target|Act|(Err%)|[%Out]", stg1MtrData);
		SmartDashboard.putString("Stg 2 RPM Target|Act|(Err%)|[%Out]", stg2MtrData);
		
		String magicCarpetMtrInOut = "?";
		if(_magicCarpetMtrTargetVBus == MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND) {
			magicCarpetMtrInOut = "IN";
		} else if (_magicCarpetMtrTargetVBus == (MAGIC_CARPET_TARGET_PERCENTVBUS_COMMAND * -1.0)){
			magicCarpetMtrInOut = "OUT";
		} else if (_magicCarpetMtrTargetVBus == 0.0){
			magicCarpetMtrInOut = "off";
		}
		
		String highRollerMtrInOut = "?";
		if(_highRollerMtrTargetVBus == HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND) {
			highRollerMtrInOut = "IN";
		} else if (_highRollerMtrTargetVBus == (HIGH_ROLLER_TARGET_PERCENTVBUS_COMMAND * -1.0)){
			highRollerMtrInOut = "OUT";
		} else if (_highRollerMtrTargetVBus == 0.0){
			highRollerMtrInOut = "off";
		}
		
		String highSpeedLaneMtrInOut= "?";
		if(_highSpeedInfeedLaneMtrTargetVBus == HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND) {
			highSpeedLaneMtrInOut = "IN";
		} else if(_highSpeedInfeedLaneMtrTargetVBus == HIGH_SPEED_INFEED_LANE_TARGET_PERCENTVBUS_COMMAND * -1.0) {
			highSpeedLaneMtrInOut = "OUT";
		} else if (_highSpeedInfeedLaneMtrTargetVBus == 0.0){
			highSpeedLaneMtrInOut = "off";
		}
		
		//Display Current magicCarpet Infeed HighRoller MtrsData		
		shooterFeederMtrsData = String.format("%s (%.0f%%) | %s (%.0f%%) | %s (%.0f%%)", 
																magicCarpetMtrInOut,
																_magicCarpetMtrTargetVBus * 100.0,
																highRollerMtrInOut,
																_highRollerMtrTargetVBus * 100.0,
																highSpeedLaneMtrInOut,
																_highSpeedInfeedLaneMtrTargetVBus * 100.0);
		
		SmartDashboard.putString("(MC | HR | HSL)", shooterFeederMtrsData);
		
		//Display Current Actuator Value
		actuatorData = String.format( "%.3f", _currentSliderPosition); //Outputs "Max" and "Min" at respective values
		
		if(_currentSliderPosition >= MAX_THRESHOLD_ACTUATOR) {
			actuatorData = actuatorData + " (MAX)";
		}
		else if(_currentSliderPosition <= MIN_THRESHOLD_ACTUATOR) {
			actuatorData = actuatorData + " (MIN)";
		}
		
		SmartDashboard.putString("Actuator Position", actuatorData);
		String suffix = "";
		if(_shooterTable.get_IsAtLowerEntry()) {
			suffix = "(1st)";
		}
		else if (_shooterTable.get_IsAtUpperEntry()) {
			suffix = "(Last)";
		}
		
		// currentShooterTableValues
		currentShooterTableValues = String.format("[#%d] %s | In:%.1f  |S:%.3f | M1:%d RPM | M2:%d RPM | %s", 
				_currentShooterTableEntry.Index,
				suffix,
				_currentShooterTableEntry.DistanceInInches,
				_currentShooterTableEntry.SliderPosition,
				_currentShooterTableEntry.Stg1MotorRPM,
				_currentShooterTableEntry.Stg2MotorRPM,
				_currentShooterTableEntry.Description);
		
		
	    	// limit spamming
	    	if((new Date().getTime() - _lastDebugWriteTimeMSec) > 1000) {
	    		System.out.println(currentShooterTableValues);
	    		// reset last time
	    		_lastDebugWriteTimeMSec = new Date().getTime();
	    	}

		
		SmartDashboard.putString("ShooterTable", currentShooterTableValues);
		SmartDashboard.putString("Distance", _currentShooterTableEntry.Description);
		
		// Light will come on when the shooter is turned on. To keep Mikey from running it the whole match!! :) 
		SmartDashboard.putBoolean("Is Shooter Running?", _isShooterMotorsReentrantRunning);
	}

	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%f", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%f", getStg1ActualRPM()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		logData.AddData("Stg1Mtr:%VBus", String.format("%.2f%%", getStg1CurrentPercentVBus()));
			
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%f", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%f", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		logData.AddData("Stg2Mtr:%VBus", String.format("%.2f%%", getStg2CurrentPercentVBus()));

		logData.AddData("Actuator Position", String.format("%.3f", _currentSliderPosition));
	}
}