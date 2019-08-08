package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem{
	public static Climber _instance = new Climber();
	
	public static Climber getInstance() {
		return _instance;
	}
	
	private TalonSRX _climberMtr;
	
	// define class level working variables
	private double _currentPercentVBusCmd;
	private double _climberMotorCurrent;
	private long _timeWhenMotorExceededThreshhold;
	private long _elapsedTimeSinceMotorCurrentExceededMaxThreshholdInMSec;
	private boolean _isClimberMotorStalled;
	private boolean _wasLastCycleOverMax;
	private boolean _isClimbing;
	
	// define class level constants
	private static final double CLIMBER_MAX_CURRENT = 20.0;
	private static final double MAX_TIME_OVER_THRESHHOLD = 315;
	public static final double CLIMBER_MOTOR_VBUS = -0.80;
	public static final double CLIMBER_MOTOR_HIGH_VBUS = -1.0;
	public static final double CLIMBER_MOTOR_LOW_VBUS = -0.40;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	private Climber() {
		_climberMtr = new TalonSRX(RobotMap.CLIMBER_CAN_BUS_ADDR);
		//_climberMtr.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);	// open loop throttle
		_climberMtr.setNeutralMode(NeutralMode.Coast);							// default to brake mode DISABLED *
    	//_climberMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	//_climberMtr.reverseSensor(false);  							// do not invert encoder feedback
		_climberMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_climberMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    	//_climberMtr.reverseOutput(true);
		
		//_isClimbing = false;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// Simple single direction, 2 speed mode controlled by joystick
	public void RunMotorUsingJoyStick(double joyStickCmd) {
		// push joystick up (0 => -1.0)
		if(joyStickCmd < -0.8) {
			// more than 1/2 way, climb @ high speed
			_climberMtr.set(ControlMode.PercentOutput, CLIMBER_MOTOR_HIGH_VBUS);
		}
		else if(joyStickCmd < -0.05) {
			// more than 1/2 way, climb @ low speed
			_climberMtr.set(ControlMode.PercentOutput, CLIMBER_MOTOR_LOW_VBUS);
		}
		else {
			_climberMtr.set(ControlMode.PercentOutput, 0.0);
		}
		
		
	}
	
	// This method starts the climber when the button is pressed
	public void RunClimberReentrant() {
		RunMotor(CLIMBER_MOTOR_VBUS);
		_isClimbing = true;
	}
		
	// This is the main drive method
	private void RunMotor(double percentVBusCmd){
		_climberMotorCurrent = _climberMtr.getOutputCurrent();
		
		if (Math.abs(_climberMotorCurrent) >= CLIMBER_MAX_CURRENT) {	
			if (_wasLastCycleOverMax == false) {
				_wasLastCycleOverMax = true;
				_timeWhenMotorExceededThreshhold = System.currentTimeMillis();
			}
			
			// Time you've been in this part of the code equals duration between time when motor exceeded and now
			_elapsedTimeSinceMotorCurrentExceededMaxThreshholdInMSec 
					= System.currentTimeMillis() - _timeWhenMotorExceededThreshhold;
			
			System.out.println("Time " + _elapsedTimeSinceMotorCurrentExceededMaxThreshholdInMSec
								+  "  i: " + _climberMotorCurrent);
			
			if (_elapsedTimeSinceMotorCurrentExceededMaxThreshholdInMSec >= MAX_TIME_OVER_THRESHHOLD) {
				_climberMtr.set(ControlMode.PercentOutput,0.0);
				_isClimberMotorStalled = true;
			}
		} else {
			_wasLastCycleOverMax = false;
		}
		
		if (!_isClimberMotorStalled) {
			// send cmd to mtr controller
			_currentPercentVBusCmd = percentVBusCmd;
			_climberMtr.set(ControlMode.PercentOutput, percentVBusCmd);
		}	
	}
	
	public void SetUpClimberStatus() {
		// This is called in Teleop Init, and it resets the climbing mechanism. 
		_isClimberMotorStalled = false;
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	
	public boolean getIsClimbing() {
		return _isClimbing;
	}
	
	private double getActualMotorCurrent() {
		return _climberMtr.getOutputCurrent();
	}
	
	private double getActualPercentVBus() {
		return GeneralUtilities.RoundDouble((_climberMtr.getMotorOutputVoltage() / _climberMtr.getBusVoltage()), 2);
	}
	
	// Per the request of the drive team
	private String getIsClimberBuckets() {
		if (_isClimberMotorStalled) {
			return "BUCKETS!";
		} else {
			return "";
		}
	}

	@Override
	public void stop() {
		_climberMtr.set(ControlMode.PercentOutput, 0.0);
		_isClimbing = false;
	}

	@Override
	public void zeroSensors() {
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber("Climber Motor Current", getActualMotorCurrent());
		SmartDashboard.putString(" ", getIsClimberBuckets());
	}

	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("ClimberMtr:Cmd_%VBus", String.format("%.2f", _currentPercentVBusCmd));
		logData.AddData("ClimberMtr:Act_%VBus", String.format("%.2f", getActualPercentVBus()));
		
		logData.AddData("ClimberMtr:Thold_Mtr_I", String.format("%.2f", CLIMBER_MAX_CURRENT));
		
		logData.AddData("ClimberMtr:Act_Mtr_I", String.format("%.2f", getActualMotorCurrent()));
		logData.AddData("ClimberMtr:OMax_Msec", String.format("%d", _elapsedTimeSinceMotorCurrentExceededMaxThreshholdInMSec));
	}
}