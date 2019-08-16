package org.usfirst.frc.team4028.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.LogData;

/*import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;*/

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHandler extends Subsystem {
	public static GearHandler _instance = new GearHandler();
	
	public static GearHandler getInstance() {
		return _instance;
	}
	
	// define class level public constants
	public static final double INFEED_TARGET_CMD = -0.5;
	public static final double OUTFEED_TARGET_CMD = 0.5;
	
	// define class level variables for Robot objects
	private TalonSRX _gearTiltMotor;
	private TalonSRX _gearInfeedMotor;
	
	// define class level private working variables
	private double _targetPositionRotations;
	private long _flapAnnReentrantRunningMsec;
	
	// --------------------------------------------------------
	// define Tilt Motor PID constants
	private static final int 	TILT_PID_P_PROFILE = 0;
	private static final double TILT_PID_P_CONSTANT = 1.0; //1.6
	private static final double TILT_PID_I_CONSTANT = 0.0;
	private static final double TILT_PID_D_CONSTANT = 50.0;
	private static final double TILT_PID_RAMP_RATE = 0.1;
		
	private static final double TILT_MAX_V_DOWN_TILT = +3.0; // Down is positive (RIP MAXIMUM...)
	private static final double TILT_MAX_V_UP_TILT = -6.0;
	// --------------------------------------------------------
	
	// --------------------------------------------------------
	// define Working variables and constants for homing the tilt axix
	private enum GEAR_TILT_HOMING_STATE {
		UNDEFINED,
		MOVING_TO_HOME,
		AT_HOME,
		TIMEOUT,
		ZEROED
	}
	
	private enum GEAR_TILT_MOVE_LAST_TARGET_POSITION {
		UNDEFINED,
		MOVING_TO_SCORING_POSITION,
		MOVING_TO_HOME,
		MOVING_TO_FLOOR
	}
	
	private static final double GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 0;
	private static final double GEAR_TILT_SCORING_POSITION_IN_ROTATIONS = 0.1;
	private static final int GEAR_TILT_SCORING_POSITION_IN_NU = 400;
	private static final double GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS = 00.48;
	private static final int GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_NU = 1000;
	private static final double TARGET_DEADBAND = 122;
	
	private static final double GEAR_MOVE_TO_HOME_VELOCITY_CMD = -0.20;   //set
	private static final long GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC = 5000;
	private String _gearTiltState;
	
	private long _gearTiltAxisStateStartTime;
	private GEAR_TILT_HOMING_STATE _gearTiltAxisZeroCurrentState;
	private GEAR_TILT_MOVE_LAST_TARGET_POSITION _gearTiltMoveLastTargetPosition;
	private boolean _isLastTiltMoveToFloorCallComplete;

	// --------------------------------------------------------
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	private GearHandler() {
		// Tilt Motor
		_gearTiltMotor = new TalonSRX(RobotMap.GEAR_TILT_CAN_BUS_ADDR);
		_gearTiltMotor.configFactoryDefault();
		//_gearTiltMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_gearTiltMotor.setNeutralMode(NeutralMode.Coast);;							// default to brake mode DISABLED
		_gearTiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);// set encoder to be feedback device
		_gearTiltMotor.setSensorPhase(false);
		_gearTiltMotor.selectProfileSlot(TILT_PID_P_PROFILE,0);
		_gearTiltMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
		
		_gearTiltMotor.configNominalOutputForward(0.0f, 0);
		_gearTiltMotor.configNominalOutputReverse(0.0f, 0);
		_gearTiltMotor.configPeakOutputForward(TILT_MAX_V_DOWN_TILT, 0);
		_gearTiltMotor.configPeakOutputReverse(TILT_MAX_V_UP_TILT, 0); 

      
      //_gearTiltMotor.config_kF(TILT_PID_P_PROFILE, , timeoutMs);
      _gearTiltMotor.config_kP(TILT_PID_P_PROFILE, TILT_PID_P_CONSTANT);
      _gearTiltMotor.config_kI(TILT_PID_P_PROFILE, TILT_PID_I_CONSTANT, 100);
      _gearTiltMotor.config_kD(TILT_PID_P_PROFILE, TILT_PID_D_CONSTANT);

		// Infeed Motor
		_gearInfeedMotor = new TalonSRX(RobotMap.GEAR_INFEED_CAN_BUS_ADDR);
		//_gearInfeedMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_gearInfeedMotor.setNeutralMode(NeutralMode.Coast);							// default to brake mode DISABLED
		
		
		ZeroGearTiltAxisInit();
	}

	//============================================================================================
	// Methods follow
	//============================================================================================	
	
    public void ZeroGearTiltAxisInit() {
		_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.UNDEFINED;   	
		
		// snapshot the current time so we can enforce the timeout
		_gearTiltAxisStateStartTime = System.currentTimeMillis();
    	
    	// did we start on the limit switch? (remember switch is normally closed!)
		if(getIsOnTiltHomeLimtSwitch()) {
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [AT_HOME]", false);
		} else {
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.MOVING_TO_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [MOVING_TO_HOME]", false);
		}
    }
	
	// Re-entrant method that will zero the Tilt Axis
    public void ZeroGearTiltAxisReentrant() {
    	switch(_gearTiltAxisZeroCurrentState) {    					
    		case MOVING_TO_HOME:
    			// are we on the limit switch? (remember switch is normally closed!
    			if(getIsOnTiltHomeLimtSwitch()) {
    				_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
    				DriverStation.reportWarning("TiltAxis (Zero State) [MOVING_TO_HOME] ==> [AT_HOME]", false);
    			} else {
    				// check for timeout
    				long elapsedTime = System.currentTimeMillis() - _gearTiltAxisStateStartTime;
    				if (elapsedTime < GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC) {
						DriverStation.reportWarning("GettingArcadeDrCmd",true);
						_gearTiltMotor.set(ControlMode.PercentOutput, GEAR_MOVE_TO_HOME_VELOCITY_CMD);
    				} else {
    					_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.TIMEOUT;
    					DriverStation.reportWarning("TiltAxis (Zero State) [MOVING_TO_HOME] ==> [TIMEOUT]", false);
    				}		
    			}
    			break;
    			
    		case AT_HOME:
    			// chg to PID-Position mode
				if(_gearTiltMotor.getControlMode() == ControlMode.PercentOutput) 
				{
					_gearTiltMotor.set(ControlMode.Position, 0);
    			}
    			
    			// reset encoder position
    			
    			_gearTiltMotor.setSelectedSensorPosition(0);
    			// set current target position to be the home position
    			_gearTiltMotor.setSelectedSensorPosition(0);
    			
    			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.ZEROED;
    			_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.MOVING_TO_HOME;
    			DriverStation.reportWarning("TiltAxis (Zero State) [AT_HOME] ==> [ZEROED]", false);
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportWarning("Gear Tilt Zero Timed Out", false);
    			break;
    			
    		case UNDEFINED:
    			DriverStation.reportWarning("Gear Tilt Zero State Undefined", false);
    			break;
    	}
    }
    
    public void MoveGearToHomePosition() {
    	MoveTiltAxisPIDP(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    	_isLastTiltMoveToFloorCallComplete = true;
    	
    	DriverStation.reportWarning("Move Gear To Home Position", false);
    }
    
    public void MoveGearToScorePosition() {
		//MoveTiltAxisPIDP(GEAR_TILT_SCORING_POSITION_IN_ROTATIONS);
		MoveTiltAxisPIDP(GEAR_TILT_SCORING_POSITION_IN_NU);
    	_isLastTiltMoveToFloorCallComplete = true;
    	
    	//DriverStation.reportWarning("Move Gear To Score Position", false);
    }
    
    public void MoveGearToFloorPositionReentrant() {
		if(_gearTiltMotor.getSelectedSensorPosition() >= (GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_NU - TARGET_DEADBAND))
		{
			// gravity fall to floor
			MoveTiltAxisVBus(0.0);
			_isLastTiltMoveToFloorCallComplete = true;
		} else {
			MoveTiltAxisPIDP(GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_NU);
			_isLastTiltMoveToFloorCallComplete = false;
		}
		
		DriverStation.reportWarning("Move Gear To Floor Position", false);
    }
    
    public void MoveTiltAxisPIDP (double positionCmd) {
    	_gearTiltMotor.set(ControlMode.Position, positionCmd);
    }

	public void MoveTiltAxisVBus(double percentVBusCmd) {
		MoveTiltAxisVBus(percentVBusCmd, true);
	}
    
	public void MoveTiltAxisVBus(double percentVBusCmd, boolean isUseRawCmd) {
		
		if(isUseRawCmd) {
			_gearTiltMotor.set(ControlMode.PercentOutput, percentVBusCmd);			
		} else {
			// limit max speed to 25%	=>  * 0.25
			_gearTiltMotor.set(ControlMode.PercentOutput, percentVBusCmd * 0.25);	
		}
		
		// cancel any button move that was in process
		_isLastTiltMoveToFloorCallComplete = true;
	}
	
	public void SpinInfeedWheelsVBus(double percentVBusCmd) {
		// invert motor command		=>	* -1.0
		// limit max speed to 50%	=>  * 0.50
		_gearInfeedMotor.set(ControlMode.PercentOutput, percentVBusCmd * -1.0 * 0.50);
	}
	
	private  String getTiltPosition() {
		if((_gearTiltMotor.getControlMode() == ControlMode.PercentOutput) 
				&& (Math.abs(_gearTiltMotor.getMotorOutputPercent()) > 0.0)) {
			_gearTiltState = "Joystick";
		}
		else if ((_gearTiltAxisZeroCurrentState != GEAR_TILT_HOMING_STATE.ZEROED)) {
			_gearTiltState = "Unknown";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition()) <= TARGET_DEADBAND) {
			_gearTiltState = "Home";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition() - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND) {
			_gearTiltState = "Scoring";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition() - GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS) > 0) {
			_gearTiltState = "Floor";
		}
		else {
			_gearTiltState = "Unknown.";
		}
		
		return _gearTiltState;
	}
	
	public void FlapAnnReentrant()
	{	
		if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 1600)	{		// 800 ; 500
			MoveGearToScorePosition();
		}
		//else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 4000) {	// 800 ; 500
			// pause
		//}
		else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 2000) {	// 40; 20; 40
			MoveGearToHomePosition();
		} 
		//else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 8000) {	// 800 ; 500
			// pause
		//}
		else {
			_flapAnnReentrantRunningMsec = System.currentTimeMillis();
		}
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	private boolean getIsOnTiltHomeLimtSwitch() {
		// remember switch is normally closed!
		return ! _gearTiltMotor.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	public boolean hasTiltAxisBeenZeroed() {
		if (_gearTiltAxisZeroCurrentState == GEAR_TILT_HOMING_STATE.ZEROED) {
			return true;
		} else {
			return false;
		}
	}
	
	//allows robot class to check if we are in moving to floor mode so it can keep calling
	public GEAR_TILT_MOVE_LAST_TARGET_POSITION get_gearTiltMoveToPosition() {
		return _gearTiltMoveLastTargetPosition;
	}
	
	public boolean IsGearInScoringPosition() {
		if(Math.abs(_gearTiltMotor.getSelectedSensorPosition() - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean getIsLastTiltMoveToFloorCallComplete() {
		return _isLastTiltMoveToFloorCallComplete;
	}

	@Override
	public void stop() {
		MoveTiltAxisVBus(0.0);
		_gearInfeedMotor.set(ControlMode.PercentOutput, 0.0);
	}

	@Override
	public void zeroSensors() {
		_gearTiltMotor.getSelectedSensorPosition(0);
	}

	@Override
	public void outputToSmartDashboard() {
		//%s - insert a string
		//%d - insert a signed integer (decimal)
		//%f - insert a real number, standard notation
		
		String gearTiltMtrData = "?";
		// we only really knwo position after we have zeroed
		if(_gearTiltAxisZeroCurrentState == GEAR_TILT_HOMING_STATE.ZEROED) {
			gearTiltMtrData = String.format("%s (%.3f)", getTiltPosition(), _gearTiltMotor.getSelectedSensorPosition());
		} else {
			gearTiltMtrData = String.format("%s (%s)", getTiltPosition(), "???");
		}
		SmartDashboard.putString("Gear Tilt Position", gearTiltMtrData);
		
		SmartDashboard.putString("Gear Tilt State", getTiltPosition());
		
		String gearInFeedMtrData = "?";
		if(Math.abs(_gearInfeedMotor.getMotorOutputVoltage()) > 0) {
			gearInFeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_gearInfeedMotor.getMotorOutputVoltage() / _gearInfeedMotor.getBusVoltage())* 100);
		} else {
			gearInFeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}
		
		SmartDashboard.putString("Gear In/OutFeed Cmd", gearInFeedMtrData);	
	}

	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("Gear:TiltPos", String.format("%.2f", _gearTiltMotor.getSelectedSensorPosition()));
		logData.AddData("Gear:Tilt%VBus", String.format("%.4f", (_gearTiltMotor.getMotorOutputVoltage()) / _gearTiltMotor.getBusVoltage()));
		logData.AddData("Gear:Outfeed%Vbus", String.format("%.2f", _gearTiltMotor.getMotorOutputPercent()));
	}
}