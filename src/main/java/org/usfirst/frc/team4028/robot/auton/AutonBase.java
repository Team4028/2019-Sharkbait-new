package org.usfirst.frc.team4028.robot.auton;

import org.usfirst.frc.team4028.robot.auton.actions.Action;

public abstract class AutonBase {
	protected boolean _active = false;
	
	public abstract void routine();
	
	public void run() {
		_active = true;
		routine();
		done();
	}
	
	public void done() {
		
	}
	
	public void stop() {
		_active = false;
	}
	
	public boolean isActive() {
		return _active;
	}
	
	public void runAction(Action action) {
		action.start();
		while (isActive() && !action.isFinished()) {
			action.update();
		}
		action.done();
	}
}