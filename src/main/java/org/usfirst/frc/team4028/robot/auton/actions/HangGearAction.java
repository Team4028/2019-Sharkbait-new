package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.controllers.HangGearController;

public class HangGearAction implements Action{
	HangGearController _hangGear = HangGearController.getInstance();
	
	private boolean _isFinished;
	
	public HangGearAction() {
		
	}

	@Override
	public void start() {
		_hangGear.Initialize();
	}

	@Override
	public void update() {
		_isFinished = _hangGear.ExecuteRentrant();
	}

	@Override
	public void done() {
	}

	@Override
	public boolean isFinished() {
		return _isFinished;
	}
}
