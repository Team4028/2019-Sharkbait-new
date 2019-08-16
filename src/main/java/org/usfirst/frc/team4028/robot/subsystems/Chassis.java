package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.constants.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis extends Subsystem{
	public static Chassis _instance = new Chassis();

	public static Chassis getInstance() {
		return _instance;
	}
	
	// define class level variables for Robot objects
	private TalonSRX _leftDriveMaster, _leftDriveSlave, _rightDriveMaster, _rightDriveSlave;
	private RobotDrive _robotDrive;				// this supports arcade/tank style drive controls
	private DoubleSolenoid _shifterSolenoid;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	
	// acc/dec variables
	private boolean _isAccelDecelEnabled;
	private double _currentThrottleCmdScaled, _previousThrottleCmdScaled;
	private double _currentThrottleCmdAccDec, _previousThrottleCmdAccDec;
	
	private double _arcadeDriveThrottleCmdAdj;
	private double _arcadeDriveTurnCmdAdj;
	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOTAL_TIME_SECS = 0.8;
	
	private static final double _turnSpeedScalingFactor = 0.7;
	
	// motion magic constants
	private static final double P_GAIN = 4.0;
	private static final double I_GAIN = 0.0;
	private static final double D_GAIN = 95.0;	
	private static final double F_GAIN = 0.52;
	private static final int MAX_ACCELERATION = 1200; // 200 RPM / S
	private static final int MAX_VELOCITY = 150; // 200 RPM
	
	// Gearbox Ratios: 1:3 encoder shaft, 34:50 output shaft
	
	// define public enums exposed by this class
	public enum GearShiftPosition {
		UNKNOWN,
		HIGH_GEAR,
		LOW_GEAR
	}	
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public Chassis() {
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
		_leftDriveMaster = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_BUS_ADDR);
		_leftDriveMaster.configFactoryDefault();
		
    	_leftDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder); // set encoder to be feedback device
    	//_leftDriveMaster.configEncoderCodesPerRev(1097);
		_leftDriveMaster.setSensorPhase(false);  							// do not invert encoder feedback
		
		_leftDriveMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_leftDriveMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
		
    	_leftDriveMaster.configMotionAcceleration(MAX_ACCELERATION);
    	_leftDriveMaster.configMotionCruiseVelocity(MAX_VELOCITY);
    
		_leftDriveMaster.config_kF(0, F_GAIN); 
		_leftDriveMaster.config_kP(0, P_GAIN); 
		_leftDriveMaster.config_kI(0, I_GAIN); 
		_leftDriveMaster.config_kD(0, D_GAIN);

		_leftDriveSlave = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE1_CAN_BUS_ADDR);
		   _leftDriveSlave.configFactoryDefault();
		   _leftDriveSlave.follow(_leftDriveMaster);	// set this mtr ctrlr as a slave
		
		_leftDriveSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_leftDriveSlave.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
		_rightDriveMaster = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_BUS_ADDR);
		_rightDriveMaster.configFactoryDefault();
    	_rightDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
		
    	_rightDriveMaster.setSensorPhase(false);							// do not invert encoder feedback
		_rightDriveMaster.setInverted(true);
		
		
		_rightDriveMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_rightDriveMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

		_rightDriveMaster.configMotionAcceleration(MAX_ACCELERATION);
    	_rightDriveMaster.configMotionCruiseVelocity(MAX_VELOCITY);
    
		_rightDriveMaster.config_kF(0, F_GAIN); 
		_rightDriveMaster.config_kP(0, P_GAIN); 
		_rightDriveMaster.config_kI(0, I_GAIN); 
		_rightDriveMaster.config_kD(0, D_GAIN);

		_rightDriveSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE1_CAN_BUS_ADDR);
		_rightDriveSlave.configFactoryDefault();
		_rightDriveSlave.setInverted(true);
		_rightDriveSlave.follow(_rightDriveMaster);	// set this mtr ctrlr as a slave
		
		
		_rightDriveSlave.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   		_rightDriveSlave.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    	  	
    	//====================
    	// Shifter
    	//====================
    	_shifterSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_BUS_ADDR, RobotMap.SHIFTER_SOLENOID_EXTEND_PCM_PORT, RobotMap.SHIFTER_SOLENOID_RETRACT_PCM_PORT);
    	
    	//====================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in "2 motor per side setup, other motors follow master as slaves 
    	//_robotDrive = new RobotDrive(_leftDriveMaster, _rightDriveMaster);
    	//_robotDrive.setSafetyEnabled(false);
    	
    	//_robotDrive.setNeutralMode(NeutralMode.Brake); // Disable motors on drive talons
    
    
    	//set default scaling factor
    	_driveSpeedScalingFactorClamped = 1.0;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// This is the (arcade) main drive method
	public void ArcadeDrive(double newThrottleCmdRaw, double newTurnCmdRaw) {
		
		// calc scaled throttle cmds
		double newThrottleCmdScaled = newThrottleCmdRaw * _driveSpeedScalingFactorClamped;
		double newTurnCmdScaled = newTurnCmdRaw * _turnSpeedScalingFactor;
		
		// if the cmd just chg'd reset 
		if(newThrottleCmdScaled != _previousThrottleCmdScaled) {
			_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			_lastCmdChgTimeStamp = System.currentTimeMillis();
		}
			
		// if acc/dec mode is enabled
		if(_isAccelDecelEnabled) {
			_previousThrottleCmdAccDec = _currentThrottleCmdAccDec;
			//DriverStation.reportWarning("GettingArcadeDrCmd",true);
			
			//implement speed scaling
			_arcadeDriveThrottleCmdAdj = calcAccelDecelThrottleCmd(_currentThrottleCmdScaled, _previousThrottleCmdScaled, _lastCmdChgTimeStamp);
			
			_currentThrottleCmdAccDec = _arcadeDriveThrottleCmdAdj;
			
			if(Math.abs(_arcadeDriveThrottleCmdAdj - _currentThrottleCmdScaled) < 0.1) {
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
			}
		} else {
			_arcadeDriveThrottleCmdAdj = newThrottleCmdScaled;
		}
		
		_arcadeDriveTurnCmdAdj = newTurnCmdScaled;
		
		// send cmd to mtr controllers
		_leftDriveMaster.set(ControlMode.PercentOutput, _arcadeDriveThrottleCmdAdj - _arcadeDriveTurnCmdAdj);
		_rightDriveMaster.set(ControlMode.PercentOutput, _arcadeDriveThrottleCmdAdj + _arcadeDriveTurnCmdAdj);
	}
	
	public void TankDrive(double leftCmd, double rightCmd) {
		
		_leftDriveMaster.set(ControlMode.PercentOutput, leftCmd);
		_rightDriveMaster.set(ControlMode.PercentOutput, rightCmd);
	}
	
	public void SetMotionMagicTargetPosition(double leftPosition, double rightPosition) {

		_leftDriveMaster.set(ControlMode.MotionMagic, leftPosition);
		_rightDriveMaster.set(ControlMode.MotionMagic, rightPosition);
	}
	
	public void EnableBrakeMode(boolean isEnabled) {
		_leftDriveMaster.setNeutralMode(NeutralMode.Brake);
		_leftDriveSlave.setNeutralMode(NeutralMode.Brake);
		_rightDriveMaster.setNeutralMode(NeutralMode.Brake);
		_rightDriveSlave.setNeutralMode(NeutralMode.Brake);
	}
	
	/* public void EnablePercentVBusMode() {
		if (_leftDriveMaster.getControlMode() != TalonControlMode.PercentVbus) {
			_leftDriveMaster.changeControlMode(TalonControlMode.PercentVbus);
			_rightDriveMaster.changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	
	public void EnableMotionMagicMode() {
		if (_leftDriveMaster.getControlMode() != ControlMode.MotionMagic) {
			_leftDriveMaster.controlM;
			_rightDriveMaster.changeControlMode(TalonControlMode.MotionMagic);
		}
	} */
	
	// shifts between high & low gear
	public void ShiftGear(GearShiftPosition gear) {
		// send cmd to to solenoids
		switch(gear) {
			case HIGH_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION;
				
    			//DriverStation.reportWarning("Shift into HIGH gear", false);
				break;
			
			case LOW_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION;
				
    			//DriverStation.reportWarning("Shift into LOW gear", false);
				break;
		}
	}
	
	public void ToggleShiftGear() {
		if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) {
			ShiftGear(GearShiftPosition.LOW_GEAR);
		} else {	
			ShiftGear(GearShiftPosition.HIGH_GEAR);
		}
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	
	public void setDriveSpeedScalingFactor(double speedScalingFactor) {
		// for safety, clamp the scaling factor to max of +1, -1
		if (speedScalingFactor > 1.0) {
			speedScalingFactor = 1.0;
		}
		else if (speedScalingFactor < -1.0){
			speedScalingFactor = -1.0;
		}
		
		_driveSpeedScalingFactorClamped = speedScalingFactor;
	}
	
	public void setIsAccDecModeEnabled(boolean isEnabled) {
		_isAccelDecelEnabled = isEnabled;
		DriverStation.reportWarning("===== Acc/Dec Mode Enabled? " + isEnabled, false);
	}
	
	public boolean getIsAccDecModeEnabled() {
		return _isAccelDecelEnabled;
	}
	
	public double getLeftEncoderCurrentPosition() {
		return _leftDriveMaster.getSelectedSensorPosition();
	}
	
	public double getLeftEncoderCurrentVelocity() {
		return (_leftDriveMaster.getSelectedSensorVelocity()/7.5);
	}
	
	public double getRightEncoderCurrentPosition() {
		return _rightDriveMaster.getSelectedSensorPosition();
	}
	
	public double getRightEncoderCurrentVelocity() {
		return (_rightDriveMaster.getSelectedSensorVelocity()/7.5);
	}
	
	@Override
	public void stop() {
		EnableBrakeMode(true);
		ArcadeDrive(0.0, 0.0);
		ShiftGear(GearShiftPosition.HIGH_GEAR);
		setIsAccDecModeEnabled(true);
		setDriveSpeedScalingFactor(1.0);
	}
	
	@Override
	public void zeroSensors() {
		_leftDriveMaster.setSelectedSensorPosition(0);
		_rightDriveMaster.setSelectedSensorPosition(0);
	}
	
	@Override
	public void outputToSmartDashboard() {
		String chassisDriveGearPosition = "";
		if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION) {
			chassisDriveGearPosition = "HIGH_GEAR";
		} 
		else if (_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION) {
			chassisDriveGearPosition = "LOW_GEAR";
		} else {
			chassisDriveGearPosition = "UNKNOWN";
		}
		
		SmartDashboard.putString("Driving Gear", chassisDriveGearPosition);
		SmartDashboard.putNumber("Left Position", getLeftEncoderCurrentPosition());
		SmartDashboard.putNumber("Right Position", getRightEncoderCurrentPosition());
		SmartDashboard.putNumber("Left Velocity", getLeftEncoderCurrentVelocity());
		SmartDashboard.putNumber("Right Velocity", getRightEncoderCurrentVelocity());
	}
	
	@Override
	public void updateLogData(LogData logData) {
		logData.AddData("Chassis:LeftDriveMtrSpd", String.format("%.2f", _leftDriveMaster.getMotorOutputPercent()));
		logData.AddData("Chassis:LeftDriveMtr%VBus", String.format("%.2f", _leftDriveMaster.getMotorOutputVoltage()/_leftDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:LeftDriveMtrPos", String.format("%.0f", _leftDriveMaster.getSelectedSensorPosition()));
		
		logData.AddData("Chassis:RightDriveMtrSpd", String.format("%.2f", _rightDriveMaster.getMotorOutputPercent()));
		logData.AddData("Chassis:RightDriveMtr%VBus", String.format("%.2f", _rightDriveMaster.getMotorOutputVoltage()/_rightDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:RightDriveMtrPos", String.format("%.0f", _rightDriveMaster.getSelectedSensorPosition()));
	}
	
	//============================================================================================
	// Utility Helper Methods
	//============================================================================================
	// implement s-curve accel / decel
	private double calcAccelDecelThrottleCmd(double currentThrottleCmd, double previousThrottleCmd, long lastCmdChgTimeStamp) {
		double accDecMidpointTimeSecs = ACC_DEC_TOTAL_TIME_SECS / 2.0;    // a

        double minusK = -1.0 * ACC_DEC_RATE_FACTOR;
        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - _lastCmdChgTimeStamp) / 1000.0; // x
        double xMinusA = elapsedSecsSinceLastChg - accDecMidpointTimeSecs;

        double scaleFactor = 1.0 / ( 1.0 + Math.exp(minusK * xMinusA) );

        // finally calc the adj cmd
        double accDecCmd = previousThrottleCmd + ((_currentThrottleCmdScaled - previousThrottleCmd) * scaleFactor);
        
        return accDecCmd;
	}
}