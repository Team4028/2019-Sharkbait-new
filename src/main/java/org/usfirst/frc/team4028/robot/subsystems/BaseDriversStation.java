package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.LogitechF310;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.Joystick;

// this class encapsulates all interactions with the DriversStation 
//	- It support Is pressed & Just pressed functionality
//  - it should be reuseable across seasons, robots & projects
//  - every project should implement a class that inherits from this class with 
//		property accessors with more specific names
//
//------------------------------------------------------
//Rev		By		 	D/T				Desc
//===		========	===========		=================================
//1.0		TomB		12.Mar.2017		Added support for engineering gamepad 
//------------------------------------------------------
// ===========================================================================
//	  TALK TO SEBAS or MR. BRUNS BEFORE you edit this file !
// ===========================================================================
abstract class BaseDriversStation {
	// ===================================
	// define robot objects for Driver & Operator station gamepads
	// ===================================
	private Joystick _driverGamepad;
	private Joystick _operatorGamepad;
	private Joystick _engineeringGamepad;
	
	// ===================================
	// define working & state variables
	// ===================================
	private DriversStationInputs _currentValues;
	private DriversStationInputs _previousValues;
	
	private final double JOYSTICK_THRESHHOLD = 0.05;

	//============================================================================================
	// constructors follow
	//============================================================================================
	protected BaseDriversStation(int driverGamePadUsbPort, int operatorGamePadUsbPort, int engineerimgGamePadUsbPort) {
		_driverGamepad = new Joystick(driverGamePadUsbPort);				// std Logitech F310 Gamepad  
		_operatorGamepad = new Joystick(operatorGamePadUsbPort);			// std Logitech F310 Gamepad  
		_engineeringGamepad = new Joystick(engineerimgGamePadUsbPort);		// std Logitech F310 Gamepad  
		
		
		if(_driverGamepad == null) {
			DriverStation.reportError("DriverGamepad not present!", false);
		}
		if(_operatorGamepad == null) {
			DriverStation.reportError("OperatorGamepad not present!", false);
		}
		if(_engineeringGamepad == null) {
			DriverStation.reportError("EngineeringGamepad not present!", false);
		}
		
		//if(!getIsGamePadConnected(_engineeringGamepad)) {
		//	_engineeringGamepad = null;
		//}
	}

	// Refreshes the locally cached copy of the Current (this scan) Driver's Station Input Values
	public void ReadCurrentScanCycleValues() {
		// always set previousValues object to null (so that object can be gc'd)
		_previousValues = null;
		
		if(_currentValues != null) {
			// if we have _currentValues (From the last scan) push them into _previosuValues
			_previousValues = _currentValues;
		}
		
		// update _currentValues by reading from the gamepads
		_currentValues = new DriversStationInputs();
		
		if(_previousValues == null) {
			// if we DO NOT have _previousValues (From the last scan) set it equal to _currentValues
			_previousValues = _currentValues;
		}
	}
	
	
	// ======================================================
	// Public Property Accessors
	// ======================================================
	
	private static boolean getIsGamePadConnected(Joystick gamePad) {
		boolean isGamepadConnected = false;
		try
		{
			HIDType joyStickType = gamePad.getType();
			isGamepadConnected = true;
		}
		catch (Exception ex) {
			//System.out.println("_driverGamepad not connected!");
		}
		
		return isGamepadConnected;
	}
	
	// === driver buttons ===============================================
	protected boolean getIsDriverGreenBtnAJustPressed() {
		return(_currentValues.getIsDriverGreenBtnAPressed()
    				&& !_previousValues.getIsDriverGreenBtnAPressed());
	}
	
	protected boolean getIsDriverRedBtnBJustPressed() {
		return(_currentValues.getIsDriverRedBtnBPressed()
    				&& !_previousValues.getIsDriverRedBtnBPressed());
	}
	
	protected boolean getIsDriverBlueBtnXJustPressed() {
		return(_currentValues.getIsDriverBlueBtnXPressed()
    				&& !_previousValues.getIsDriverBlueBtnXPressed());
	}
	
	protected boolean getIsDriverYellowBtnYJustPressed() {
		return(_currentValues.getIsDriverYellowBtnYPressed()
    				&& !_previousValues.getIsDriverYellowBtnYPressed());
	}

	protected boolean getIsDriverLeftBumperBtnJustPressed() {
		return(_currentValues.getIsDriverLeftBumperBtnPressed()
    				&& !_previousValues.getIsDriverLeftBumperBtnPressed());
	}
	
	protected boolean getIsDriverRightBumperBtnJustPressed() {
		return(_currentValues.getIsDriverRightBumperBtnPressed()
    				&& !_previousValues.getIsDriverRightBumperBtnPressed());
	}

	protected boolean getIsDriverBackBtnJustPressed() {
		return(_currentValues.getIsDriverBackBtnPressed()
    				&& !_previousValues.getIsDriverBackBtnPressed());
	}

	protected boolean getIsDriverStartBtnJustPressed() {
		return (_currentValues.getIsDriverStartBtnPressed()
    				&& !_previousValues.getIsDriverStartBtnPressed());
	}
	
	protected boolean getIsDriverPovUpBtnJustPressed() {
		return (_currentValues.getIsDriverPovBtnPressed()
    				&& !_previousValues.getIsDriverPovBtnPressed());
	}
	
	protected boolean getIsDriverLeftThumbstickBtnJustPressed() {
		return (_currentValues.getIsDriverLeftThumbstickPressed()
				&& !_previousValues.getIsDriverLeftThumbstickPressed());
	}
	
	protected boolean getIsDriverRightThumbstickBtnJustPressed() {
		return (_currentValues.getIsDriverRightThumbstickPressed() 
				&& !_previousValues.getIsDriverRightThumbstickPressed());
	}
	
	// Instantaneous Driver Buttons
	protected boolean getIsDriverGreenBtnAPressed() {
		return _currentValues.getIsDriverGreenBtnAPressed();
	}
	
	protected boolean getIsDriverRedBtnBPressed() {
		return _currentValues.getIsDriverRedBtnBPressed();
	}
	
	protected boolean getIsDriverBlueBtnXPressed() {
		return _currentValues.getIsDriverBlueBtnXPressed();
    }
	
	protected boolean getIsDriverYellowBtnYPressed() {
		return _currentValues.getIsDriverYellowBtnYPressed();
    }

	protected boolean getIsDriverLeftBumperBtnPressed() {
		return _currentValues.getIsDriverLeftBumperBtnPressed();
    }
	
	protected boolean getIsDriverRightBumperBtnPressed() {
		return _currentValues.getIsDriverRightBumperBtnPressed();
    };

    protected boolean getIsDriverBackBtnPressed() {
		return _currentValues.getIsDriverBackBtnPressed();
    }

	protected boolean getIsDriverStartBtnPressed() {
		return _currentValues.getIsDriverStartBtnPressed();
	}
	
	// === operator buttons ===============================================
	
	protected boolean getIsOperatorGreenBtnAJustPressed() {
		return(_currentValues.getIsOperatorGreenBtnAPressed()
    				&& !_previousValues.getIsOperatorGreenBtnAPressed());
	}
	
	protected boolean getIsOperatorRedBtnBJustPressed() {
		return(_currentValues.getIsOperatorRedBtnBPressed()
    				&& !_previousValues.getIsOperatorRedBtnBPressed());
	}
	
	protected boolean getIsOperatorBlueBtnXJustPressed() {
		return(_currentValues.getIsOperatorBlueBtnXPressed()
    				&& !_previousValues.getIsOperatorBlueBtnXPressed());
	}
	
	protected boolean getIsOperatorYellowBtnYJustPressed() {
		return(_currentValues.getIsOperatorYellowBtnYPressed()
    				&& !_previousValues.getIsOperatorYellowBtnYPressed());
	}

	protected boolean getIsOperatorLeftBumperBtnJustPressed() {
		return(_currentValues.getIsOperatorLeftBumperBtnPressed()
    				&& !_previousValues.getIsOperatorLeftBumperBtnPressed());
	}
	
	protected boolean getIsOperatorRightBumperBtnJustPressed() {
		return(_currentValues.getIsOperatorRightBumperBtnPressed()
    				&& !_previousValues.getIsOperatorRightBumperBtnPressed());
	}

	protected boolean getIsOperatorBackBtnJustPressed() {
		return(_currentValues.getIsOperatorBackBtnPressed()
    				&& !_previousValues.getIsOperatorBackBtnPressed());
	}

	protected boolean getIsOperatorStartBtnJustPressed() {
		return (_currentValues.getIsOperatorStartBtnPressed()
    				&& !_previousValues.getIsOperatorStartBtnPressed());
	}
	
	protected boolean getIsOperatorPovBtnJustPressed() {
		return (_currentValues.getIsOperatorPovBtnPressed()
				&& !_previousValues.getIsOperatorPovBtnPressed());
	}
	
	protected boolean getIsOperatorLeftThumbstickBtnJustPressed() {
		return (_currentValues.getIsOperatorLeftThumbstickPressed()
				&& !_previousValues.getIsOperatorLeftThumbstickPressed());
	}
	
	protected boolean getIsOperatorRightThumbstickBtnJustPressed() {
		return (_currentValues.getIsOperatorRightThumbstickPressed() 
				&& !_previousValues.getIsOperatorRightThumbstickPressed());
	}	
	
	protected boolean getIsOperatorLeftTriggerJustPressed() {
		return (_currentValues.getIsOperatorLeftTriggerPressed() 
				&& !_previousValues.getIsOperatorLeftTriggerPressed());
	}
	
	//Instantaneous Operator Buttons
	protected boolean getIsOperatorGreenBtnAPressed() {
		return(_currentValues.getIsOperatorGreenBtnAPressed());
	}
	
	protected boolean getIsOperatorRedBtnBPressed() {
		return(_currentValues.getIsOperatorRedBtnBPressed());
	}
	
	protected boolean getIsOperatorBlueBtnXPressed() {
		return(_currentValues.getIsOperatorBlueBtnXPressed());
	}
	
	protected boolean getIsOperatorYellowBtnYPressed() {
		return(_currentValues.getIsOperatorYellowBtnYPressed());
	}

	protected boolean getIsOperatorLeftBumperBtnPressed() {
		return(_currentValues.getIsOperatorLeftBumperBtnPressed());
	}
	
	protected boolean getIsOperatorRightBumperBtnPressed() {
		return(_currentValues.getIsOperatorRightBumperBtnPressed());
	}

	protected boolean getIsOperatorBackBtnPressed() {
		return(_currentValues.getIsOperatorBackBtnPressed());
	}

	protected boolean getIsOperatorStartBtnPressed() {
		return (_currentValues.getIsOperatorStartBtnPressed());
	}
	
	protected boolean getIsOperatorLeftTriggerPressed() {
		return (_currentValues.getIsOperatorLeftTriggerPressed());
	}
		
	//Instantaneous Engineering Buttons
	protected boolean getIsEngineeringGreenBtnAPressed() {
		return(_currentValues.getIsEngineeringGreenBtnAPressed());
	}
	
	protected boolean getIsEngineeringRedBtnBPressed() {
		return(_currentValues.getIsEngineeringRedBtnBPressed());
	}
	
	protected boolean getIsEngineeringBlueBtnXPressed() {
		return(_currentValues.getIsEngineeringBlueBtnXPressed());
	}
	
	protected boolean getIsEngineeringYellowBtnYPressed() {
		return(_currentValues.getIsEngineeringYellowBtnYPressed());
	}

	protected boolean getIsEngineeringLeftBumperBtnPressed() {
		return(_currentValues.getIsEngineeringLeftBumperBtnPressed());
	}
	
	protected boolean getIsEngineeringRightBumperBtnPressed() {
		return(_currentValues.getIsEngineeringRightBumperBtnPressed());
	}

	protected boolean getIsEngineeringBackBtnPressed() {
		return(_currentValues.getIsEngineeringBackBtnPressed());
	}

	protected boolean getIsEngineeringStartBtnPressed() {
		return (_currentValues.getIsEngineeringStartBtnPressed());
	}
	
	// === engineering just pressed  buttons ===============================================
	protected boolean getIsEngineeringGreenBtnAJustPressed() {
		return(_currentValues.getIsEngineeringGreenBtnAPressed()
    				&& !_previousValues.getIsEngineeringGreenBtnAPressed());
	}
	
	protected boolean getIsEngineeringRedBtnBJustPressed() {
		return(_currentValues.getIsEngineeringRedBtnBPressed()
    				&& !_previousValues.getIsEngineeringRedBtnBPressed());
	}
	
	protected boolean getIsEngineeringBlueBtnXJustPressed() {
		return(_currentValues.getIsEngineeringBlueBtnXPressed()
    				&& !_previousValues.getIsEngineeringBlueBtnXPressed());
	}
	
	protected boolean getIsEngineeringYellowBtnYJustPressed() {
		return(_currentValues.getIsEngineeringYellowBtnYPressed()
    				&& !_previousValues.getIsEngineeringYellowBtnYPressed());
	}

	protected boolean getIsEngineeringLeftBumperBtnJustPressed() {
		return(_currentValues.getIsEngineeringLeftBumperBtnPressed()
    				&& !_previousValues.getIsEngineeringLeftBumperBtnPressed());
	}
	
	protected boolean getIsEngineeringRightBumperBtnJustPressed() {
		return(_currentValues.getIsEngineeringRightBumperBtnPressed()
    				&& !_previousValues.getIsEngineeringRightBumperBtnPressed());
	}

	protected boolean getIsEngineeringBackBtnJustPressed() {
		return(_currentValues.getIsEngineeringBackBtnPressed()
    				&& !_previousValues.getIsEngineeringBackBtnPressed());
	}

	protected boolean getIsEngineeringStartBtnJustPressed() {
		return (_currentValues.getIsEngineeringStartBtnPressed()
    				&& !_previousValues.getIsEngineeringStartBtnPressed());
	}
	
	protected boolean getIsEngineeringPovUpBtnJustPressed() {
		return (_currentValues.getIsEngineeringPovBtnPressed()
    				&& !_previousValues.getIsEngineeringPovBtnPressed());
	}
	
	protected boolean getIsEngineeringLeftThumbstickBtnJustPressed() {
		return (_currentValues.getIsEngineeringLeftThumbstickPressed()
				&& !_previousValues.getIsEngineeringLeftThumbstickPressed());
	}
	
	protected boolean getIsEngineeringRightThumbstickBtnJustPressed() {
		return (_currentValues.getIsEngineeringRightThumbstickPressed() 
				&& !_previousValues.getIsEngineeringRightThumbstickPressed());
	}
	
	
	// === driver joysticks ===============================================
	protected double getDriverLeftXAxisCmd() {
		return _currentValues.getDriverLeftXAxisCmd();
	}
	
	protected double getDriverLeftYAxisCmd() {
		return _currentValues.getDriverLeftYAxisCmd();
	}

	protected double getDriverLeftTriggerCmd() {
		return _currentValues.getDriverLeftTriggerCmd();
	}

	protected double getDriverRightTriggerCmd() {
		return _currentValues.getDriverRightTriggerCmd();
	}

	protected double getDriverRightXAxisCmd() {
		return _currentValues.getDriverRightXAxisCmd();
	}

	protected double getDriverRightYAxisCmd() {
		return _currentValues.getDriverRightYAxisCmd();
	}
	
	// === operator joysticks ===============================================
	protected double getOperatorLeftXAxisCmd() {
		return _currentValues.getOperatorLeftXAxisCmd();
	}
	
	protected double getOperatorLeftYAxisCmd() {
		return _currentValues.getOperatorLeftYAxisCmd();
	}

	protected double getOperatorLeftTriggerCmd() {
		return _currentValues.getOperatorLeftTriggerCmd();
	}

	protected double getOperatorRightTriggerCmd() {
		return _currentValues.getOperatorRightTriggerCmd();
	}

	protected double getOperatorRightXAxisCmd() {
		return _currentValues.getOperatorRightXAxisCmd();
	}

	protected double getOperatorRightYAxisCmd() {
		return _currentValues.getOperatorRightYAxisCmd();
	}
	
	// === engineering joysticks ===============================================
	protected double getEngineeringLeftXAxisCmd() {
		return _currentValues.getEngineeringLeftXAxisCmd();
	}
	
	protected double getEngineeringLeftYAxisCmd() {
		return _currentValues.getEngineeringLeftYAxisCmd();
	}

	protected double getEngineeringLeftTriggerCmd() {
		return _currentValues.getEngineeringLeftTriggerCmd();
	}

	protected double getEngineeringRightTriggerCmd() {
		return _currentValues.getEngineeringRightTriggerCmd();
	}

	protected double getEngineeringRightXAxisCmd() {
		return _currentValues.getEngineeringRightXAxisCmd();
	}

	protected double getEngineeringRightYAxisCmd() {
		return _currentValues.getEngineeringRightYAxisCmd();
	}
	
	/************************************************************
	 * Immutable class to hold data read from the gamepads
	 ************************************************************/
	public final class DriversStationInputs {
		// =============================
		// private backing fields
		// =============================
		
		// digital inputs
    	private final boolean _isDriverGreenBtnAPressed;
    	private final boolean _isDriverRedBtnBPressed;
    	private final boolean _isDriverBlueBtnXPressed;
    	private final boolean _isDriverYellowBtnYPressed;
    	private final boolean _isDriverLeftBumperBtnPressed;
    	private final boolean _isDriverRightBumperBtnPressed;
    	private final boolean _isDriverBackBtnPressed;
    	private final boolean _isDriverStartBtnPressed;
    	private final boolean _isDriverPovBtnPressed;
    	private final boolean _isDriverLeftThumbstickBtnPressed;
    	private final boolean _isDriverRightThumbstickBtnPressed;
    	
    	private final boolean _isOperatorGreenBtnAPressed;
    	private final boolean _isOperatorRedBtnBPressed;
    	private final boolean _isOperatorBlueBtnXPressed;
    	private final boolean _isOperatorYellowBtnYPressed;
    	private final boolean _isOperatorLeftBumperBtnPressed;
    	private final boolean _isOperatorRightBumperBtnPressed;
    	private final boolean _isOperatorBackBtnPressed;
    	private final boolean _isOperatorStartBtnPressed;
    	private final boolean _isOperatorPovBtnPressed;
    	private final boolean _isOperatorLeftThumbstickBtnPressed;
    	private final boolean _isOperatorRightThumbstickBtnPressed;
    	
    	private final boolean _isEngineeringGreenBtnAPressed;
    	private final boolean _isEngineeringRedBtnBPressed;
    	private final boolean _isEngineeringBlueBtnXPressed;
    	private final boolean _isEngineeringYellowBtnYPressed;
    	private final boolean _isEngineeringLeftBumperBtnPressed;
    	private final boolean _isEngineeringRightBumperBtnPressed;
    	private final boolean _isEngineeringBackBtnPressed;
    	private final boolean _isEngineeringStartBtnPressed;
    	private final boolean _isEngineeringPovBtnPressed;
    	private final boolean _isEngineeringLeftThumbstickBtnPressed;
    	private final boolean _isEngineeringRightThumbstickBtnPressed;
    	
    	// analog inputs
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	private final double _driverLeftXAxisCmd;
    	private final double _driverLeftYAxisCmd;
    	private final double _driverLeftTriggerCmd;
    	private final double _driverRightTriggerCmd;
    	private final double _driverRightXAxisCmd;
    	private final double _driverRightYAxisCmd;
    	
    	private final double _operatorLeftXAxisCmd;
    	private final double _operatorLeftYAxisCmd;
    	private final double _operatorLeftTriggerCmd;
    	private final double _operatorRightTriggerCmd;
    	private final double _operatorRightXAxisCmd;
    	private final double _operatorRightYAxisCmd;
    	
    	private final double _engineeringLeftXAxisCmd;
    	private final double _engineeringLeftYAxisCmd;
    	private final double _engineeringLeftTriggerCmd;
    	private final double _engineeringRightTriggerCmd;
    	private final double _engineeringRightXAxisCmd;
    	private final double _engineeringRightYAxisCmd;
		
		/**
		 * Create an entirely new instance by reading from the Gamepads
		 * @param driverGamepad
		 * @param operatorGamepad
		 */
		protected DriversStationInputs() {
	    	// ==========================
	    	// get values from the gamepads
	    	// ==========================
			// digital inputs			
			if(_driverGamepad != null && getIsGamePadConnected(_driverGamepad)) {
				_isDriverGreenBtnAPressed = _driverGamepad.getRawButton(LogitechF310.GREEN_BUTTON_A);
		    	_isDriverRedBtnBPressed = _driverGamepad.getRawButton(LogitechF310.RED_BUTTON_B);
		    	_isDriverBlueBtnXPressed = _driverGamepad.getRawButton(LogitechF310.BLUE_BUTTON_X);
		    	_isDriverYellowBtnYPressed = _driverGamepad.getRawButton(LogitechF310.YELLOW_BUTTON_Y);
		    	_isDriverLeftBumperBtnPressed = _driverGamepad.getRawButton(LogitechF310.LEFT_BUMPER);
		    	_isDriverRightBumperBtnPressed = _driverGamepad.getRawButton(LogitechF310.RIGHT_BUMPER);
		    	_isDriverBackBtnPressed = _driverGamepad.getRawButton(LogitechF310.BACK_BUTTON);
		    	_isDriverStartBtnPressed = _driverGamepad.getRawButton(LogitechF310.START_BUTTON);
		    	_isDriverPovBtnPressed = (_driverGamepad.getPOV() > -1);
		    	_isDriverLeftThumbstickBtnPressed = _driverGamepad.getRawButton(LogitechF310.LEFT_THUMBSTICK);
		    	_isDriverRightThumbstickBtnPressed = _driverGamepad.getRawButton(LogitechF310.RIGHT_THUMBSTICK);
		    	
		    	// analog inputs
		    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
		    	_driverLeftXAxisCmd = _driverGamepad.getRawAxis(LogitechF310.LEFT_X_AXIS);
		    	_driverLeftYAxisCmd = _driverGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
		    	_driverLeftTriggerCmd = _driverGamepad.getRawAxis(LogitechF310.LEFT_TRIGGER);
		    	_driverRightTriggerCmd = _driverGamepad.getRawAxis(LogitechF310.RIGHT_TRIGGER);
		    	_driverRightXAxisCmd = _driverGamepad.getRawAxis(LogitechF310.RIGHT_X_AXIS);
		    	_driverRightYAxisCmd = _driverGamepad.getRawAxis(LogitechF310.RIGHT_Y_AXIS);
			}
			else {
				_isDriverGreenBtnAPressed = false;
		    	_isDriverRedBtnBPressed = false;
		    	_isDriverBlueBtnXPressed = false;
		    	_isDriverYellowBtnYPressed = false;
		    	_isDriverLeftBumperBtnPressed = false;
		    	_isDriverRightBumperBtnPressed = false;
		    	_isDriverBackBtnPressed = false;
		    	_isDriverStartBtnPressed = false;
		    	_isDriverPovBtnPressed = false;
		    	_isDriverLeftThumbstickBtnPressed = false;
		    	_isDriverRightThumbstickBtnPressed = false;
		    	
		    	// analog inputs
		    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
		    	_driverLeftXAxisCmd = 0.0;
		    	_driverLeftYAxisCmd = 0.0;
		    	_driverLeftTriggerCmd = 0.0;
		    	_driverRightTriggerCmd = 0.0;
		    	_driverRightXAxisCmd = 0.0;
		    	_driverRightYAxisCmd = 0.0;	
			}
			
			if(_operatorGamepad != null && getIsGamePadConnected(_operatorGamepad))
			{
				_isOperatorGreenBtnAPressed = _operatorGamepad.getRawButton(LogitechF310.GREEN_BUTTON_A);
		    	_isOperatorRedBtnBPressed = _operatorGamepad.getRawButton(LogitechF310.RED_BUTTON_B);
		    	_isOperatorBlueBtnXPressed = _operatorGamepad.getRawButton(LogitechF310.BLUE_BUTTON_X);
		    	_isOperatorYellowBtnYPressed = _operatorGamepad.getRawButton(LogitechF310.YELLOW_BUTTON_Y);
		    	_isOperatorLeftBumperBtnPressed = _operatorGamepad.getRawButton(LogitechF310.LEFT_BUMPER);
		    	_isOperatorRightBumperBtnPressed = _operatorGamepad.getRawButton(LogitechF310.RIGHT_BUMPER);
		    	_isOperatorBackBtnPressed = _operatorGamepad.getRawButton(LogitechF310.BACK_BUTTON);
		    	_isOperatorStartBtnPressed = _operatorGamepad.getRawButton(LogitechF310.START_BUTTON);
		    	_isOperatorPovBtnPressed = (_operatorGamepad.getPOV() > -1);
		    	_isOperatorLeftThumbstickBtnPressed = _operatorGamepad.getRawButton(LogitechF310.LEFT_THUMBSTICK);
		    	_isOperatorRightThumbstickBtnPressed = _operatorGamepad.getRawButton(LogitechF310.RIGHT_THUMBSTICK);
		    	
		    	_operatorLeftXAxisCmd = _operatorGamepad.getRawAxis(LogitechF310.LEFT_X_AXIS);
		    	_operatorLeftYAxisCmd = _operatorGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
		    	_operatorLeftTriggerCmd = _operatorGamepad.getRawAxis(LogitechF310.LEFT_TRIGGER);
		    	_operatorRightTriggerCmd = _operatorGamepad.getRawAxis(LogitechF310.RIGHT_TRIGGER);
		    	_operatorRightXAxisCmd = _operatorGamepad.getRawAxis(LogitechF310.RIGHT_X_AXIS);
		    	_operatorRightYAxisCmd = _operatorGamepad.getRawAxis(LogitechF310.RIGHT_Y_AXIS);
			}
			else 			{
				_isOperatorGreenBtnAPressed = false;
		    	_isOperatorRedBtnBPressed = false;
		    	_isOperatorBlueBtnXPressed = false;
		    	_isOperatorYellowBtnYPressed = false;
		    	_isOperatorLeftBumperBtnPressed = false;
		    	_isOperatorRightBumperBtnPressed = false;
		    	_isOperatorBackBtnPressed = false;
		    	_isOperatorStartBtnPressed = false;
		    	_isOperatorPovBtnPressed = false;
		    	_isOperatorLeftThumbstickBtnPressed = false;
		    	_isOperatorRightThumbstickBtnPressed = false;
		    	
		    	_operatorLeftXAxisCmd = 0.0;
		    	_operatorLeftYAxisCmd = 0.0;
		    	_operatorLeftTriggerCmd = 0.0;
		    	_operatorRightTriggerCmd = 0.0;
		    	_operatorRightXAxisCmd = 0.0;
		    	_operatorRightYAxisCmd = 0.0;
			}
			
			if(_engineeringGamepad != null && getIsGamePadConnected(_engineeringGamepad))
			{
				_isEngineeringGreenBtnAPressed = _engineeringGamepad.getRawButton(LogitechF310.GREEN_BUTTON_A);
		    	_isEngineeringRedBtnBPressed = _engineeringGamepad.getRawButton(LogitechF310.RED_BUTTON_B);
		    	_isEngineeringBlueBtnXPressed = _engineeringGamepad.getRawButton(LogitechF310.BLUE_BUTTON_X);
		    	_isEngineeringYellowBtnYPressed = _engineeringGamepad.getRawButton(LogitechF310.YELLOW_BUTTON_Y);
		    	_isEngineeringLeftBumperBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.LEFT_BUMPER);
		    	_isEngineeringRightBumperBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.RIGHT_BUMPER);
		    	_isEngineeringBackBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.BACK_BUTTON);
		    	_isEngineeringStartBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.START_BUTTON);
		    	_isEngineeringPovBtnPressed = (_engineeringGamepad.getPOV() > -1);
		    	_isEngineeringLeftThumbstickBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.LEFT_THUMBSTICK);
		    	_isEngineeringRightThumbstickBtnPressed = _engineeringGamepad.getRawButton(LogitechF310.RIGHT_THUMBSTICK);
		    	  
		    	_engineeringLeftXAxisCmd = _engineeringGamepad.getRawAxis(LogitechF310.LEFT_X_AXIS);
		    	_engineeringLeftYAxisCmd = _engineeringGamepad.getRawAxis(LogitechF310.LEFT_Y_AXIS);
		    	_engineeringLeftTriggerCmd = _engineeringGamepad.getRawAxis(LogitechF310.LEFT_TRIGGER);
		    	_engineeringRightTriggerCmd = _engineeringGamepad.getRawAxis(LogitechF310.RIGHT_TRIGGER);
		    	_engineeringRightXAxisCmd = _engineeringGamepad.getRawAxis(LogitechF310.RIGHT_X_AXIS);
		    	_engineeringRightYAxisCmd = _engineeringGamepad.getRawAxis(LogitechF310.RIGHT_Y_AXIS);
			}
			else {
				_isEngineeringGreenBtnAPressed = false;
		    	_isEngineeringRedBtnBPressed = false;
		    	_isEngineeringBlueBtnXPressed = false;
		    	_isEngineeringYellowBtnYPressed = false;
		    	_isEngineeringLeftBumperBtnPressed = false;
		    	_isEngineeringRightBumperBtnPressed = false;
		    	_isEngineeringBackBtnPressed = false;
		    	_isEngineeringStartBtnPressed = false;
		    	_isEngineeringPovBtnPressed = false;
		    	_isEngineeringLeftThumbstickBtnPressed = false;
		    	_isEngineeringRightThumbstickBtnPressed = false;
		    	  
		    	_engineeringLeftXAxisCmd = 0.0;
		    	_engineeringLeftYAxisCmd = 0.0;
		    	_engineeringLeftTriggerCmd = 0.0;
		    	_engineeringRightTriggerCmd = 0.0;
		    	_engineeringRightXAxisCmd = 0.0;
		    	_engineeringRightYAxisCmd = 0.0;
			}
		}
		

		
		// === driver buttons ====================================
		public boolean getIsDriverGreenBtnAPressed() {
    		return _isDriverGreenBtnAPressed;
    	}
		
		public boolean getIsDriverRedBtnBPressed() {
    		return _isDriverRedBtnBPressed;
    	}
		
		public boolean getIsDriverBlueBtnXPressed() {
    		return _isDriverBlueBtnXPressed;
    	}
		
		public boolean getIsDriverYellowBtnYPressed() {
    		return _isDriverYellowBtnYPressed;
    	}

		public boolean getIsDriverLeftBumperBtnPressed() {
    		return _isDriverLeftBumperBtnPressed;
    	}

		public boolean getIsDriverRightBumperBtnPressed() {
    		return _isDriverRightBumperBtnPressed;
    	}
		
		public boolean getIsDriverBackBtnPressed() {
    		return _isDriverBackBtnPressed;
    	}

		public boolean getIsDriverStartBtnPressed() {
    		return _isDriverStartBtnPressed;
    	}
		
		public boolean getIsDriverPovBtnPressed() {
			return _isDriverPovBtnPressed;
		}
		
		public boolean getIsDriverLeftThumbstickPressed() {
			return _isDriverLeftThumbstickBtnPressed;
		}
		
		public boolean getIsDriverRightThumbstickPressed() {
			return _isDriverRightThumbstickBtnPressed;
		}
		
		// === operator buttons ====================================
		public boolean getIsOperatorGreenBtnAPressed() {
    		return _isOperatorGreenBtnAPressed;
    	}
		
		public boolean getIsOperatorRedBtnBPressed() {
    		return _isOperatorRedBtnBPressed;
    	}
		
		public boolean getIsOperatorBlueBtnXPressed() {
    		return _isOperatorBlueBtnXPressed;
    	}
		
		public boolean getIsOperatorYellowBtnYPressed() {
    		return _isOperatorYellowBtnYPressed;
    	}

		public boolean getIsOperatorLeftBumperBtnPressed() {
    		return _isOperatorLeftBumperBtnPressed;
    	}

		public boolean getIsOperatorRightBumperBtnPressed() {
    		return _isOperatorRightBumperBtnPressed;
    	}
		
		public boolean getIsOperatorBackBtnPressed() {
    		return _isOperatorBackBtnPressed;
    	}

		public boolean getIsOperatorStartBtnPressed() {
    		return _isOperatorStartBtnPressed;
    	}
		
		public boolean getIsOperatorPovBtnPressed() {
			return _isOperatorPovBtnPressed;
		}
		
		public boolean getIsOperatorLeftThumbstickPressed() {
			return _isOperatorLeftThumbstickBtnPressed;
		}
		
		public boolean getIsOperatorRightThumbstickPressed() {
			return _isOperatorRightThumbstickBtnPressed;
		}
		
		public boolean getIsOperatorLeftTriggerPressed() {
			return (_operatorLeftTriggerCmd > 0.25);
		}
		
		// === operator buttons ====================================
		public boolean getIsEngineeringGreenBtnAPressed() {
    		return _isEngineeringGreenBtnAPressed;
    	}
		
		public boolean getIsEngineeringRedBtnBPressed() {
    		return _isEngineeringRedBtnBPressed;
    	}
		
		public boolean getIsEngineeringBlueBtnXPressed() {
    		return _isEngineeringBlueBtnXPressed;
    	}
		
		public boolean getIsEngineeringYellowBtnYPressed() {
    		return _isEngineeringYellowBtnYPressed;
    	}

		public boolean getIsEngineeringLeftBumperBtnPressed() {
    		return _isEngineeringLeftBumperBtnPressed;
    	}

		public boolean getIsEngineeringRightBumperBtnPressed() {
    		return _isEngineeringRightBumperBtnPressed;
    	}
		
		public boolean getIsEngineeringBackBtnPressed() {
    		return _isEngineeringBackBtnPressed;
    	}

		public boolean getIsEngineeringStartBtnPressed() {
    		return _isEngineeringStartBtnPressed;
    	}
		
		public boolean getIsEngineeringPovBtnPressed() {
			return _isEngineeringPovBtnPressed;
		}
		
		public boolean getIsEngineeringLeftThumbstickPressed() {
			return _isEngineeringLeftThumbstickBtnPressed;
		}
		
		public boolean getIsEngineeringRightThumbstickPressed() {
			return _isEngineeringRightThumbstickBtnPressed;
		}
		// === driver joysticks ====================================
		
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	public double getDriverLeftXAxisCmd() {
    		if(Math.abs(_driverLeftXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return (_driverLeftXAxisCmd);
    		} else {
    			return 0.0;
    		}
    	}
		
    	public double getDriverLeftYAxisCmd() {
    		if(Math.abs(_driverLeftYAxisCmd) > JOYSTICK_THRESHHOLD)
    		{
    			return _driverLeftYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getDriverLeftTriggerCmd() {
    		if(Math.abs(_driverLeftTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _driverLeftTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getDriverRightTriggerCmd() {
    		if(Math.abs(_driverRightTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _driverRightTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getDriverRightXAxisCmd() {
    		if(Math.abs(_driverRightXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _driverRightXAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}
    	
    	public double getDriverRightYAxisCmd() {
    		if(Math.abs(_driverRightYAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _driverRightYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}
    	
		// === operator joysticks ====================================
    	public double getOperatorLeftXAxisCmd() {
    		if(Math.abs(_operatorLeftXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return (_operatorLeftXAxisCmd);
    		} else {
    			return 0.0;
    		}
    	}
		
    	public double getOperatorLeftYAxisCmd() {
    		if(Math.abs(_operatorLeftYAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _operatorLeftYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getOperatorLeftTriggerCmd() {
    		if(Math.abs(_operatorLeftTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _operatorLeftTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getOperatorRightTriggerCmd() {
    		if(Math.abs(_operatorRightTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _operatorRightTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getOperatorRightXAxisCmd() {
    		if(Math.abs(_operatorRightXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _operatorRightXAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}
    	
    	public double getOperatorRightYAxisCmd() {
    		if(Math.abs(_operatorRightYAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _operatorRightYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}	
    	
		// === engineering joysticks ====================================
    	public double getEngineeringLeftXAxisCmd() {
    		if(Math.abs(_engineeringLeftXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return (_engineeringLeftXAxisCmd);
    		} else {
    			return 0.0;
    		}
    	}
		
    	public double getEngineeringLeftYAxisCmd() {
    		if(Math.abs(_engineeringLeftYAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _engineeringLeftYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getEngineeringLeftTriggerCmd() {
    		if(Math.abs(_engineeringLeftTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _engineeringLeftTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getEngineeringRightTriggerCmd() {
    		if(Math.abs(_engineeringRightTriggerCmd) > JOYSTICK_THRESHHOLD) {
    			return _engineeringRightTriggerCmd;
    		} else {
    			return 0.0;
    		}
    	}

    	public double getEngineeringRightXAxisCmd() {
    		if(Math.abs(_engineeringRightXAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _engineeringRightXAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}
    	
    	public double getEngineeringRightYAxisCmd() {
    		if(Math.abs(_engineeringRightYAxisCmd) > JOYSTICK_THRESHHOLD) {
    			return _engineeringRightYAxisCmd;
    		} else {
    			return 0.0;
    		}
    	}
	}
}