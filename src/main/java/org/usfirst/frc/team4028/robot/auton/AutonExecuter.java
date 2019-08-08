package org.usfirst.frc.team4028.robot.auton;

public class AutonExecuter {
	private AutonBase _autoMode;
	
	public void setAutoMode(AutonBase autoMode) {
		_autoMode = autoMode;
	}
	
	public void start() {
		_autoMode.run();
	}
	
	public void stop() {
		if (_autoMode != null) {
			_autoMode.stop();
		}
	}
}
