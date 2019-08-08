package org.usfirst.frc.team4028.robot.controllers;
import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.CenterGearTrajectory;
import org.usfirst.frc.team4028.robot.util.JTurn;
import org.usfirst.frc.team4028.robot.util.MotionProfile;
import org.usfirst.frc.team4028.robot.util.MoveToBoilerTrajectory;
import org.usfirst.frc.team4028.robot.util.MoveToHopperBlue_X;
import org.usfirst.frc.team4028.robot.util.MoveToHopperRed_X;
import org.usfirst.frc.team4028.robot.util.SideGearTrajectory;
import org.usfirst.frc.team4028.robot.util.TrajectoryFollower;
import org.usfirst.frc.team4028.robot.util.TwoGearLong;
import org.usfirst.frc.team4028.robot.util.TwoGearShort;
import org.usfirst.frc.team4028.robot.util.TwoGearSuperShort;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.MOTION_PROFILE;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.RoboRealmClient;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryDriveController {
	public static TrajectoryDriveController _instance = new TrajectoryDriveController();
	
	public static TrajectoryDriveController getInstance() {
		return _instance;
	}
	
	private Chassis _chassis = Chassis.getInstance();
	private NavXGyro _navX = NavXGyro.getInstance();
	private UpdaterTask _updaterTask;
	private TrajectoryFollower _leftFollower = new TrajectoryFollower();
	private TrajectoryFollower _rightFollower = new TrajectoryFollower();
	private RoboRealmClient _roboRealm = RoboRealmClient.getInstance();
	private java.util.Timer _updaterTimer;
	private double _angleDiff;
	private double _currentVisionError;
	private double _direction;
	private double _heading;
	private double _kTurnGyro;
	private double _kTurnVision = -0.006;
	private double _leftCommand;
	private double _rightCommand;
	private double _leftOutput;
	private double _rightOutput;
	private double _setVisionError;
	private double _turn;
	private double _visionTurnThreshold = 0.15;
	private double[][] _leftMotionProfile;
	private double[][] _rightMotionProfile;
	private boolean _isAutoStopEnabled = false;
	private boolean _isEnabled;
	private boolean _isUpdaterTaskRunning;
	private boolean _isVisionTrackingEnabled;
	private int _currentSegment;
	private int _trajectoryNumPoints;
	
	public TrajectoryDriveController() {
		_updaterTimer = new java.util.Timer();
		_updaterTask = new UpdaterTask();
		setIsFeedbackDisabled(false);
	}
	
	public void configureIsHighGear(boolean isHighGear) {
		if(isHighGear) {
			_leftFollower.configure(1.0, 0.0, 0.0, 0.095, 0.08); // High Gear Constants
			_rightFollower.configure(1.0, 0.0, 0.0, 0.095, 0.08);
			_kTurnGyro = -0.01;
		} else {
			_leftFollower.configure(0.2,  0.0,  0.0,  0.174,  0.025); // Low Gear Constants // 0.3, 0.0, 0.0, 0.174, 0.032
			_rightFollower.configure(0.2,  0.0,  0.0,  0.174,  0.025);
			_kTurnGyro = -0.013;
		}
	}
	
	public boolean onTarget() {
		if(_currentSegment == (_trajectoryNumPoints - 1)) {
			return true;
		} else {
			return false;
		}
	}
	
	public void loadProfile(MotionProfile motionProfile) {
		reset();
		_angleDiff = 0.0;
		_leftMotionProfile = motionProfile.getLeftProfile();
		_rightMotionProfile = motionProfile.getRightProfile();
		_heading = motionProfile.getHeading();
		_direction = motionProfile.getDirection();
		_trajectoryNumPoints = motionProfile.getLeftProfile().length;
		_chassis.ShiftGear(motionProfile.getGearPosition());
	}
	
	public void loadProfile(MOTION_PROFILE motionProfile, boolean isBlueAlliance) {
		reset();
		_angleDiff = 0.0;
		switch(motionProfile) {
			case BOILER_GEAR:
				if (isBlueAlliance) {
					_leftMotionProfile = SideGearTrajectory.LeftPoints;
					_rightMotionProfile = SideGearTrajectory.RightPoints;
					_heading = 1.0;
				} else {
					_leftMotionProfile = SideGearTrajectory.RightPoints;
					_rightMotionProfile = SideGearTrajectory.LeftPoints;
					_heading = -1.0;
				}
				_direction = 1.0;
				_trajectoryNumPoints = SideGearTrajectory.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case CENTER_GEAR:
				_leftMotionProfile = CenterGearTrajectory.LeftPoints;
				_rightMotionProfile = CenterGearTrajectory.LeftPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = CenterGearTrajectory.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
				break;
				
			case J_TURN:
				if (isBlueAlliance) {
					_leftMotionProfile = JTurn.RightPoints;
					_rightMotionProfile = JTurn.LeftPoints;
					_heading = 1.0;
				} else {
					_leftMotionProfile = JTurn.LeftPoints;
					_rightMotionProfile = JTurn.RightPoints;
					_heading = -1.0;
				}
				_direction = -1.0;
				_trajectoryNumPoints = JTurn.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case MOVE_TO_BOILER:
				if (isBlueAlliance) {
					_leftMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_heading = -1.0;
				} else {
					_leftMotionProfile = MoveToBoilerTrajectory.RightPoints;
					_rightMotionProfile = MoveToBoilerTrajectory.LeftPoints;
					_heading = 1.0;
				}
				_direction = -1.0;
				_trajectoryNumPoints = MoveToBoilerTrajectory.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case MOVE_TO_HOPPER_BLUE_X:
				_leftMotionProfile = MoveToHopperBlue_X.LeftPoints;
				_rightMotionProfile = MoveToHopperBlue_X.RightPoints;
				_heading = -1.0;
				_direction = -1.0;
				_trajectoryNumPoints = MoveToHopperBlue_X.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case MOVE_TO_HOPPER_RED_X:
				_leftMotionProfile = MoveToHopperRed_X.RightPoints;
				_rightMotionProfile = MoveToHopperRed_X.LeftPoints;
				_heading = 1.0;
				_direction = -1.0;
				_trajectoryNumPoints = MoveToHopperRed_X.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case RETRIEVAL_GEAR:
				if (isBlueAlliance) {
					_leftMotionProfile = SideGearTrajectory.RightPoints;
					_rightMotionProfile = SideGearTrajectory.LeftPoints;
					_heading = -1.0;
				} else {
					_leftMotionProfile = SideGearTrajectory.LeftPoints;
					_rightMotionProfile = SideGearTrajectory.RightPoints;
					_heading = 1.0;
				}
				_direction = 1.0;
				_trajectoryNumPoints = SideGearTrajectory.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_LONG:
				_leftMotionProfile = TwoGearLong.LeftPoints;
				_rightMotionProfile = TwoGearLong.RightPoints;
				_direction = 1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearLong.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SHORT_FWD:
				if (isBlueAlliance) {
					_leftMotionProfile = TwoGearShort.RightPoints;
					_rightMotionProfile = TwoGearShort.LeftPoints;
					_heading = -1.0;
				} else {
					_leftMotionProfile = TwoGearShort.LeftPoints;
					_rightMotionProfile = TwoGearShort.RightPoints;
					_heading = 1.0;
				}
				_direction = 1.0;
				_trajectoryNumPoints = TwoGearShort.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SHORT_REV:
				_leftMotionProfile = TwoGearShort.LeftPoints;
				_rightMotionProfile = TwoGearShort.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearShort.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
				
			case TWO_GEAR_SUPER_SHORT:
				_leftMotionProfile = TwoGearSuperShort.LeftPoints;
				_rightMotionProfile = TwoGearSuperShort.RightPoints;
				_direction = -1.0;
				_heading = 1.0;
				_trajectoryNumPoints = TwoGearSuperShort.LeftPoints.length;
				
				_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
				break;
		}
	}
	
	public void reset() {
		_leftFollower.reset();
		_rightFollower.reset();
	}
	
	public void update(int currentSegment) {
		if (!_isEnabled) {
			DriverStation.reportWarning("Not Enabled", false);
			_chassis.TankDrive(0, 0);
		}
		
		if (onTarget()) {
			DriverStation.reportWarning("At Target", false);
			_chassis.TankDrive(0, 0);
		} else {
			double distanceL = _direction * _chassis.getLeftEncoderCurrentPosition();
			double distanceR = _direction * _chassis.getRightEncoderCurrentPosition();
			

			if(_isVisionTrackingEnabled && _roboRealm.get_isVisionDataValid()) {
				//setIsFeedbackDisabled(true);
				_currentVisionError = _roboRealm.get_Angle();
		
				if(_setVisionError != _currentVisionError) {
					_setVisionError = _currentVisionError;
				}
				_turn = _kTurnVision * _setVisionError;
				if (_turn > _visionTurnThreshold) {
					_turn = _visionTurnThreshold;
				} else if (_turn < (-1.0 * _visionTurnThreshold)) {
					_turn = -1.0 * _visionTurnThreshold;
				} else {
				}
			
			} else {
				//setIsFeedbackDisabled(false);
				double goalHeading = _leftFollower.getHeading();
				double goalHeadingInDegrees = _heading * BeefyMath.arctan(goalHeading);
				double observedHeading = _navX.getYaw();

				_turn = _kTurnGyro * (observedHeading - goalHeadingInDegrees);
			}
			
			if(_isAutoStopEnabled && _roboRealm.get_isVisionDataValid()) {
				if (!_roboRealm.get_isInGearHangPosition()) {
					_leftCommand = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
					_rightCommand = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
				} else {
					_leftCommand = 0.0;
					_rightCommand = 0.0;
					DriverStation.reportError("AUTO STOP!", false);
				}
			} else {
				_leftCommand = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
				_rightCommand = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
			}
			if (_direction == 1.0) {
				_leftOutput = GeneralUtilities.ClampValue(_leftCommand - _turn, 0.0, 1.0);
				_rightOutput = GeneralUtilities.ClampValue(_rightCommand + _turn, 0.0, 1.0);
			} else if (_direction == -1.0) {
				_leftOutput = GeneralUtilities.ClampValue(_leftCommand - _turn, -1.0, 0.0);
				_rightOutput = GeneralUtilities.ClampValue(_rightCommand + _turn, -1.0, 0.0);
			}
			_chassis.TankDrive(_leftOutput, _rightOutput);
		}
	}
	
	public void enable() {
		_leftFollower.setTrajectoryNumPoints(_trajectoryNumPoints);
		_rightFollower.setTrajectoryNumPoints(_trajectoryNumPoints);
		_leftFollower.reset();
		_rightFollower.reset();
		_chassis.zeroSensors();
		_chassis.stop();
		_navX.zeroYaw();
		_currentSegment = 0;
		_isEnabled = true;
		DriverStation.reportError("Enabled", false);
	}
	
	public void disable() { 
		_isEnabled = false; 
		_chassis.stop();
	}
	
	public boolean isEnable() { 
		return _isEnabled; 
	}
	
	public void isVisionTrackingEnabled(boolean isEnabled) {
		_isVisionTrackingEnabled = isEnabled;
	}
	
	public double getAngleDiff() {
		return _angleDiff;
	}
	
	public int getCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public double getCurrentHeading() {
		return _navX.getYaw();
	}
	
	public double getDirection() {
		return _direction;
	}
	
	public int getFollowerCurrentSegment() {
		return _currentSegment;
	}
	
	public void setIsFeedbackDisabled(boolean isDisabled) {
		_leftFollower.setIsFeedbackDisabled(isDisabled);
		_rightFollower.setIsFeedbackDisabled(isDisabled);
	}
	
	public void startTrajectoryController() {
		_isUpdaterTaskRunning = true;
		_updaterTimer.scheduleAtFixedRate(_updaterTask, 0, 20);
	}
	
	public void stopTrajectoryController() {
		_isUpdaterTaskRunning = false;
		//_updaterTimer.cancel();
	}
	
	public void OutputToSmartDashboard() {
		if(_roboRealm.get_isVisionDataValid()) {
			SmartDashboard.putNumber("Vision ", _roboRealm.get_Angle());
		}
		
		SmartDashboard.putNumber("Motor Output", _leftOutput);
	}
	
	private class UpdaterTask extends TimerTask {
		public void run() {
			while(_isUpdaterTaskRunning) {
				if (_isEnabled) {
					if (_currentSegment != (_trajectoryNumPoints - 1)) {
						update(_currentSegment);
						_currentSegment = _currentSegment + 1;
					}	
				}
				try {
					Thread.sleep(20);
				} catch(InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}