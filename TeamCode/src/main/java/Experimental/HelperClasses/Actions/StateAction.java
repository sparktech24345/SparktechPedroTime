package Experimental.HelperClasses.Actions;

import Experimental.HelperClasses.RobotState;

import static Experimental.HelperClasses.GlobalStorage.*;

public class StateAction extends Action {

    private RobotState state;
    public StateAction(boolean waitForPrevious, RobotState state) {
        this.waitForPrevious = waitForPrevious;
        this.state = state;
    }

    @Override
    public void execute() {
        started = true;
        currentRobotState = state;
        loadRobotState();
        done = true;
    }

    @Override
    public void update() {}
}
