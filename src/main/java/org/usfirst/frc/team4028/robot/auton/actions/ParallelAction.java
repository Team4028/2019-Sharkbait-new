package org.usfirst.frc.team4028.robot.auton.actions;

import java.util.ArrayList;
import java.util.List;

public class ParallelAction implements Action{
	private final ArrayList<Action> _actionList;
	
	public ParallelAction(List<Action> actionList) {
		_actionList = new ArrayList<>(actionList.size());
        for (Action action : actionList) {
            _actionList.add(action);
        }
	}

	@Override
	public void start() {
		for (Action action : _actionList) {
			action.start();
		}
	}

	@Override
	public void update() {
		for (Action action : _actionList) {
			action.update();
		}
	}

	@Override
	public void done() {
		for (Action action : _actionList) {
			action.done();
		}
	}

	@Override
	public boolean isFinished() {
		boolean isAllFinished = true;
		for (Action action : _actionList) {
			if (!action.isFinished()) {
				isAllFinished = false;
			}
		}
		
		return isAllFinished;
	}
}