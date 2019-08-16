package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.LogData;

import edu.wpi.first.wpilibj.DriverStation;

//This class implements all functionality to read Operator & Driver Gamepads
//
//------------------------------------------------------
//	Rev		By		 	D/T				Desc
//	===		========	===========		=================================
//	0		TomB		17.Feb.2017		Initial Version
//	1		TomB		18.FEb.2018		added OPERATOR_START_BUTTON			Gear Tilt ReZero
//  2		SydneyS		11.Mar.2017		add POV buttons						Reassigned driver/operator buttons
//------------------------------------------------------
//
public class DriversStation extends BaseDriversStation {
	public static DriversStation _instance = new DriversStation();
	
	public static DriversStation getInstance() {
		return _instance;
	}
	//============================================================================================
	// constructors follow
	//============================================================================================
	private DriversStation() {
		// call base class constructor
		super(RobotMap.DRIVER_GAMEPAD_USB_PORT, RobotMap.OPERATOR_GAMEPAD_USB_PORT, RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
	}
	
	
	/*
	==========================================================================
	--- Driver Joysticks --------
	DRIVER_LEFT_X_AXIS
	DRIVER_LEFT_Y_AXIS				ChassisThrottle_JoystickCmd
	DRIVER_LEFT_TRIGGER				Chassis Spin Left
	DRIVER_RIGHT_TRIGGER			Chassis Spin Right
	DRIVER_RIGHT_X_AXIS				ChassisTurn_JoystickCmd
	DRIVER_RIGHT_Y_AXIS
	
	--- Driver Buttons --------
	DRIVER_GREEN_BUTTON_A		Gear Tilt Goto Floor
	DRIVER_RED_BUTTON_B			Gear Tilt Goto Score 
	DRIVER_BLUE_BUTTON_X		Hang Gear Sequence Initiation
	DRIVER_YELLOW_BUTTON_Y		Gear Tilt GoTo Home
	DRIVER_LEFT_BUMPER			Gear Infeed
	DRIVER_RIGHT_BUMPER			Gear Outfeed
	DRIVER_BACK_BUTTON			Gear Tilt Rezero
	DRIVER_START_BUTTON			Gear Shift Toggle
	DRIVER_LEFT_THUMBSTICK
	DRIVER_RIGHT_THUMBSTICK
	DRIVER_POV					
	
							
								Gear Tilt
						
	==========================================================================
		
	--- Operator Joysticks --------
	OPERATOR_LEFT_X_AXIS			
	OPERATOR_LEFT_Y_AXIS			Fuel In/Out feed
	OPERATOR_LEFT_TRIGGER			
	OPERATOR_RIGHT_TRIGGER			
	OPERATOR_RIGHT_X_AXIS
	OPERATOR_RIGHT_Y_AXIS			Climber Speed (Slow/Fast)
	
	--- Operator Buttons --------
	OPERATOR_GREEN_BUTTON_A			Shoot Ball
	OPERATOR_RED_BUTTON_B			Shooter Slider Down
	OPERATOR_BLUE_BUTTON_X			Camera Swap
	OPERATOR_YELLOW_BUTTON_Y		Shooter Slider Up
	OPERATOR_LEFT_BUMPER			Shooter index down
	OPERATOR_RIGHT_BUMPER			Shooter Index Up
	OPERATOR_BACK_BUTTON			High Speed Lane
	OPERATOR_START_BUTTON			Run Shooter Feeder Mtrs in Reverse
	OPERATOR_LEFT_THUMBSTICK		
	OPERATOR_RIGHT_THUMBSTICK		
	OPERATOR_POV					
	==========================================================================
	*/
		
	// ======================================================
	// Public Property Accessors
	//		Implement calls into base class property accessors with 
	//		Robot function specific names
	// ======================================================
	
	// =========================================================================================================
	// DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER
	// =========================================================================================================
	
	// ===================================
	// === driver Just Pressed buttons ===
	// ===================================
	/*   <sample code>
	public boolean getIsDriver_FEATUREA_BtnJustPressed()
	{
		return super.getIsDriverGreenBtnAJustPressed();
	} 
	*/
	
	// Gear Tilt Zero
	public boolean getIsDriver_RezeroGearTilt_ButtonJustPressed() {
		return super.getIsDriverBackBtnJustPressed();
	}

	// Driving Gear Shift Toggle
	public boolean getIsDriver_ShiftDrivingGear_BtnJustPressed() {
		return super.getIsDriverStartBtnJustPressed();
	}
	
	// Gear Tilt Home
	public boolean getIsDriver_SendGearTiltToHome_BtnJustPressed() {
		return super.getIsDriverYellowBtnYJustPressed();
	}
	
	// Gear Tilt Score 
	public boolean getIsDriver_SendGearTiltToScore_BtnJustPressed() {
		return super.getIsDriverRedBtnBJustPressed();
	}
	
	// Gear Tilt Floor
	public boolean getIsDriver_SendGearTiltToFloor_BtnJustPressed() {
		return super.getIsDriverGreenBtnAJustPressed();
	}	
	
	// Gear AutoScore Sequence
	public boolean getIsDriver_StartGearScoreSequence_BtnJustPressed() {
		return super.getIsDriverBlueBtnXJustPressed();
	}
	
	public boolean getIsDriver_InfeedGear_BtnJustPressed() {
		return super.getIsDriverLeftBumperBtnJustPressed();
	}
			
	// ===================================
	// === driver Is Pressed buttons =====
	// ===================================
	/*  <sample code>
	public boolean getIsDriver_FEATUREA_BtnPressed()
	{
		return super.getIsDriverGreenBtnPressed();
	}
	*/

	// Gear Infeed
	public boolean getIsDriver_InfeedGear_BtnPressed() {
		return super.getIsDriverLeftBumperBtnPressed();
	}
	
	// Gear Outfeed
	public boolean getIsDriver_OutfeedGear_BtnPressed() {
		return super.getIsDriverRightBumperBtnPressed();
	}
	
	// Gear Tilt Floor
	public boolean getIsDriver_SendGearTiltToFloor_BtnPressed() {
		return super.getIsDriverGreenBtnAPressed();
	}	
	
	// ===================================
	// === driver Joysticks ==============
	// ===================================
	/*	<sample code>
	public double getDriver_FEATUREB_JoystickCmd()
	{
		return super.getDriverLeftXAxisCmd();
	}
	*/
	
	// Chassis Throttle
	public double getDriver_ChassisThrottle_JoystickCmd() {
		return super.getDriverLeftYAxisCmd();
	}
	
	// Chassis Turn
	public double getDriver_ChassisTurn_JoystickCmd() {
		return super.getDriverRightXAxisCmd();
	}

	// Chassis Spin Left
	public double getDriver_SpinChassisLeft_JoystickCmd() {
		return super.getDriverLeftTriggerCmd();
	}
	
	// Chassis Spin Right
	public double getDriver_SpinChassisRight_JoystickCmd() {
		return super.getDriverRightTriggerCmd();
	}
	
	public void setDriverVibrateOn(boolean isOn)
	{
		//return super.setDriverVibrateOn(isOn);
	}
	
	// =========================================================================================================
	// OPERATOR		OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	
	// =========================================================================================================
	
	
	// =====================================
	// === operator Just Pressed buttons ===
	// =====================================
	
	// High Speed Lane Toggle
	public boolean getIsOperator_ToggleHighSpeedLane_BtnJustPressed() {  
		return super.getIsOperatorBackBtnJustPressed();
	}
	
	// Slider Up
	public boolean getIsOperator_MoveShooterSliderUp_BtnJustPressed() {
		return super.getIsOperatorYellowBtnYJustPressed();
	}
	
	// Slider Down
	public boolean getIsOperator_MoveShooterSliderDown_BtnJustPressed() {
		return super.getIsOperatorRedBtnBJustPressed();
	}
		
	// Toggle Shooter Motors
	public boolean getIsOperator_ToggleShooterMotors_BtnJustPressed() {
		return super.getIsOperatorGreenBtnAJustPressed();
	}
	
	// Camera Swap
	public boolean getIsOperator_SwapCamera_BtnJustPressed() {
		return super.getIsOperatorBlueBtnXJustPressed();
	}
	
	public boolean getIsOperator_IndexShooterSettingsDown_BtnJustPressed() {
		return super.getIsOperatorLeftBumperBtnJustPressed();
	}
	
	public boolean getIsOperator_IndexShooterSettingsUp_BtnJustPressed() {
		return super.getIsOperatorRightBumperBtnJustPressed();
	}
		
	// =====================================
	// === operator Is Pressed buttons =====
	// =====================================
	public boolean getIsOperator_IndexShooterSettingsDown_BtnPressed() {
		return super.getIsOperatorLeftBumperBtnPressed();
	}
	
	public boolean getIsOperator_IndexShooterSettingsUp_BtnPressed() {
		return super.getIsOperatorRightBumperBtnPressed();
	}
	
	public boolean getIsOperator_RunShooterFeederInReverse_BtnPressed()
	{
		return super.getIsOperatorStartBtnPressed();
	}

	public boolean getIsOperator_AutoDistance_BtnPressed()
	{
		return super.getIsOperatorBackBtnPressed();
	}
	
	// =====================================
	// === operator Joysticks ==============
	// =====================================
	// Vision Auto Aim
	public boolean getIsOperator_VisionAim_BtnPressed() {
		return super.getIsOperatorLeftTriggerPressed();
	}
	
	public boolean getIsOperator_VisionAim_BtnJustPressed() {
		return super.getIsOperatorLeftTriggerJustPressed();
	}
	
	// Fuel Infeed / Outfeed
	public double getOperator_FuelInfeedOutfeed_JoystickCmd() {
		return super.getOperatorLeftYAxisCmd();
	}
	
	// Climber Speed
	public double getOperator_ClimberSpeed_JoystickCmd() {
		return super.getOperatorRightYAxisCmd();
	}
	
	public boolean getOperator_FireBall_BtnPressed() {
		// make this act like a button
		return (super.getOperatorRightTriggerCmd() > 0.1);
	}
	
	// =========================================================================================================
	// ENGINEERING	ENGINEERING	ENGINEERING	ENGINEERING	ENGINEERING	ENGINEERING	ENGINEERING	ENGINEERING	
	// =========================================================================================================
	
	// =====================================
	// === engineering Just Pressed buttons ===
	// =====================================
	
	// Stg 1 Bump RPM UP
	public boolean getIsEngineering_BumpStg1RPMUp_BtnJustPressed() {
		return super.getIsEngineeringBackBtnJustPressed();
	}
	
	// Stg 1 Bump RPM Down
	public boolean getIsEngineering_BumpStg1RPMDown_BtnJustPressed() {
		return super.getIsEngineeringLeftThumbstickBtnJustPressed();
	}
	
	// Stg 2 Bump RPM UP
	public boolean getIsEngineering_BumpStg2RPMUp_BtnJustPressed() {
		return super.getIsEngineeringStartBtnJustPressed();
	}
	
	// Stg 1 Bump RPM Down
	public boolean getIsEngineering_BumpStg2RPMDown_BtnJustPressed() {
		return super.getIsEngineeringRightThumbstickBtnJustPressed();
	}
	
	// Slider Up
	public boolean getIsEngineering_MoveShooterSliderUp_BtnJustPressed() {
		return super.getIsEngineeringYellowBtnYJustPressed();
	}
	
	// Slider Down
	public boolean getIsEngineering_MoveShooterSliderDown_BtnJustPressed() {
		return super.getIsEngineeringRedBtnBJustPressed();
	}
	
	// Toggle Shooter Motors
	public boolean getIsEngineering_ToggleShooterMotors_BtnJustPressed() {
		return super.getIsEngineeringGreenBtnAJustPressed();
	}
	
	// Fire Ball
	public boolean getEngineering_FireBall_BtnPressed() {
		// make this act like a button
		return (super.getIsEngineeringRightBumperBtnPressed());
	}
	
	// Camera Swap
	public boolean getIsEngineering_SwapCamera_BtnJustPressed() {
		return super.getIsEngineeringBlueBtnXJustPressed();
	}

	// Vision Auto Aim
	public boolean getIsEngineering_VisionAim_BtnPressed() {
		return super.getIsEngineeringLeftBumperBtnPressed();
	}
	
	
	// Joysticks
	
	// Chassis Spin Left
	public double getEngineering_SpinChassisLeft_JoystickCmd() {
		return super.getEngineeringLeftTriggerCmd();
	}
	
	// Chassis Spin Right
	public double getEngineering_SpinChassisRight_JoystickCmd() {
		return super.getEngineeringRightTriggerCmd();
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// update the Dashboard with any Drivers Station specific data values
	public void OutputToSmartDashboard() {	
	}
	
	public void UpdateLogData(LogData logData) {
	}
}