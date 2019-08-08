package org.usfirst.frc.team4028.robot.util;

import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;

public class TrajectoryFollower {
	private boolean _isFeedbackDisabled;
	private double _kp;
	private double _ki; // Likely will not be used, currently not implemented
	private double _kd;
	private double _kv;
	private double _ka;
	private double _currentHeading;
	private double _lastError;
	private double _positionOutput;
	private double _velocityOutput;
	private int _currentSegment;
	private int _trajectoryNumPoints;	
	
	public TrajectoryFollower() {}
	
	public void configure(double kp, double ki, double kd, double kv, double ka) {
		_kp = kp;
		_ki = ki;
		_kd = kd;
		_kv = kv;
		_ka = ka;
		_isFeedbackDisabled = false;
	}
	
	public void reset() {
		_lastError = 0.0;
		_currentSegment = 0;
	}
	
	public double calculate(double distanceSoFar, double[][] motionProfile, int currentSegment) {
		if (currentSegment < _trajectoryNumPoints) {
	      double error = motionProfile[currentSegment][0] - distanceSoFar;
	      if(_isFeedbackDisabled) {
	    	  _positionOutput = 0;
	      } else {
	    	  _positionOutput = _kp * error + _kd * ((error - _lastError) / motionProfile[currentSegment][4] - motionProfile[currentSegment][1]);
	      }
	      
    	  if (motionProfile[currentSegment][2] > 0.0) {
	    	  _velocityOutput = _kv * motionProfile[currentSegment][1] + _ka * motionProfile[currentSegment][2];
	      } else {
	    	  _velocityOutput = _kv * motionProfile[currentSegment][1] + 0.15 * _ka * motionProfile[currentSegment][2];
	      }
	      
	      _velocityOutput = GeneralUtilities.ClampValue(_velocityOutput, -1.0, 1.0);
	      
	      double output = _positionOutput + _velocityOutput;
	
	      _lastError = error;
	      _currentHeading = motionProfile[currentSegment][3];
	      _currentSegment = currentSegment;
	      return output;
	    } else {
	      return 0;
	    }
	}
	
	public boolean isTrajectoryFinished() {
		return _currentSegment >= _trajectoryNumPoints;
	}
	
	public void setIsFeedbackDisabled(boolean isDisabled) {
		_isFeedbackDisabled = isDisabled;
	}

	public void setTrajectoryNumPoints(int numPoints) {
		_trajectoryNumPoints = numPoints;
	}
	
	public double getHeading() {
		return _currentHeading;
	}
	
	public int getCurrentSegment() {
		return _currentSegment;
	}
}