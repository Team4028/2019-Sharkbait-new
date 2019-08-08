package org.usfirst.frc.team4028.robot.auton.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action{
	private double _startTime;
	private double _waitTime;
	
	public WaitAction(double waitTime) {
		_waitTime = waitTime;
	}

	@Override
	public void start() {
		_startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - _startTime > _waitTime;
	}
}