package org.usfirst.frc.team4028.robot.auton.modes;

import org.usfirst.frc.team4028.robot.auton.AutonBase;
import org.usfirst.frc.team4028.robot.auton.actions.RunMotionProfileAction;
import org.usfirst.frc.team4028.robot.util.CenterGearTrajectory;

public class CrossBaseLine extends AutonBase{

	@Override
	public void routine() {
		runAction(new RunMotionProfileAction(new CenterGearTrajectory()));
	}
}
