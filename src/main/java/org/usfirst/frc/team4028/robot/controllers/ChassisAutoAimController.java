package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;

public class ChassisAutoAimController {
	Chassis _chassis = Chassis.getInstance();
	NavXGyro _navX = NavXGyro.getInstance();
	
	private double _angleError;
	
	public ChassisAutoAimController() {
	}
	
	// =========================================================
	// Methods
	// =========================================================
	public void motionMagicMoveToTarget(double target) {
		
		_angleError = target - _navX.getYaw();
		
		double encoderError = BeefyMath.degreesToEncoderRotations(_angleError);
		
		double leftDriveTargetPosition = _chassis.getLeftEncoderCurrentPosition() - encoderError;
		double rightDriveTargetPosition = _chassis.getRightEncoderCurrentPosition() + encoderError;
		
		_chassis.SetMotionMagicTargetPosition(leftDriveTargetPosition, rightDriveTargetPosition);
	}
	
	public double currentHeading() {
		return _navX.getYaw();
	}
	
	public void stop() {
		_chassis.stop();
	}
	
	public boolean isOnTarget() {
		return Math.abs(_angleError) < 1.0;
	}
}