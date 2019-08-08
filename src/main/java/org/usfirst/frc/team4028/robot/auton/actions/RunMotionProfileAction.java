package org.usfirst.frc.team4028.robot.auton.actions;

import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.util.MotionProfile;

public class RunMotionProfileAction implements Action {
	TrajectoryDriveController _trajController = TrajectoryDriveController.getInstance();
	public RunMotionProfileAction(MotionProfile motionProfile) {
		_trajController.loadProfile(motionProfile);
		
	}

	@Override
	public void start() {
		_trajController.enable();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		_trajController.disable();
	}

	@Override
	public boolean isFinished() {
		
		return _trajController.onTarget();
	}
}
