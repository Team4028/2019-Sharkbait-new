package org.usfirst.frc.team4028.robot.util;

import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;

public abstract class MotionProfile {
	private static double[][] _leftProfile;
	private static double[][] _rightProfile;
	private double _heading;
	private double _direction;
	private boolean _flipProfiles;
	private GearShiftPosition _gearPosition;
	
	public MotionProfile(double heading, double direction, boolean flip, GearShiftPosition gearPosition) {
		_heading = heading;
		_direction = direction;
		_flipProfiles = flip;
		_gearPosition = gearPosition;
	}

	public double[][] getLeftProfile() {
		if (!_flipProfiles)
			return _leftProfile;
		else 
			return _rightProfile;
	}
	
	public double[][] getRightProfile() {
		if (!_flipProfiles)
			return _rightProfile;
		else 
			return _leftProfile;
	}
	
	public double getHeading() {
		return _heading;
	}
	
	public double getDirection() {
		return _direction;
	}
	
	public GearShiftPosition getGearPosition() {
		return _gearPosition;
	}
}
