package org.usfirst.frc.team4028.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.utilities.LogData;



import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallInfeed extends Subsystem {
	public static BallInfeed _instance = new BallInfeed();
	
	public static BallInfeed getInstance() {
		return _instance;
	}
	
	private TalonSRX _fuelInfeedMtr;
	private Solenoid _fuelInfeedSolenoid;
	
	private BallInfeed() {
		_fuelInfeedMtr = new TalonSRX(RobotMap.BALL_FLOOR_INFEED_MTR_CAN_BUS_ADDR);
		//_fuelInfeedMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_fuelInfeedMtr.setNeutralMode(NeutralMode.Coast);					// default to brake mode DISABLED
		_fuelInfeedMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_fuelInfeedMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);					//no limit switches
		
		_fuelInfeedSolenoid = new Solenoid(RobotMap.PCM_CAN_BUS_ADDR, RobotMap.BALL_FLOOR_INFEED_EXTEND_PCM_PORT);
	}
	
	public void InfeedFuelAndExtendSolenoid(double percentVBusCmd) {
		// run motor using joystick cmd
		_fuelInfeedMtr.set(ControlMode.PercentOutput, percentVBusCmd * -1.0);
		
		// if running fwd or reverse fire solenoid
		if(percentVBusCmd != 0) {
			_fuelInfeedSolenoid.set(true);			//extend Solenoid
		} else {
			_fuelInfeedSolenoid.set(false);			//retract Solenoid
		}
	}
	
	@Override
	public void stop() {
		_fuelInfeedSolenoid.set(false);				//retract Solenoid
		_fuelInfeedMtr.set(ControlMode.PercentOutput, 0.0);						//stop motors	
	}
	
	@Override
	public void zeroSensors() {
	}
		
	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Is Fuel Infeed Tilt Extended", _fuelInfeedSolenoid.get());
		
		String ballInfeedMtrData = "?";
		
		if(Math.abs(_fuelInfeedMtr.getMotorOutputVoltage()) > 0) {
			ballInfeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_fuelInfeedMtr.getMotorOutputVoltage() / _fuelInfeedMtr.getBusVoltage()) * 100);
		} else {
			ballInfeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}
		
		SmartDashboard.putString("Fuel Infeed", ballInfeedMtrData);
	}
	
	@Override
	public void updateLogData(LogData logData) {
	}
}