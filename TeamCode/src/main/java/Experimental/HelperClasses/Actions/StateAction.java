package Experimental.HelperClasses.Actions;

import static Experimental.HelperClasses.GlobalStorage.robotControllerInstance;

import android.util.Pair;

import java.util.function.BooleanSupplier;

public class StateAction extends Action {

    public StateAction(boolean waitForPrevious, String ComponentName, String PositionName) {
        this.waitForPrevious = waitForPrevious;
        this.Execution = () -> {
            robotControllerInstance.getComponent(ComponentName).loadState(PositionName);
        };
    }

    public StateAction(boolean waitForPrevious, String RobotStateName) {
        this.waitForPrevious = waitForPrevious;
        this.Execution = () -> {
            robotControllerInstance.loadRobotState(RobotStateName);
        };
    }

    public StateAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }

    public StateAction setDoneCondition(BooleanSupplier done) {
        this.DoneCondition = done;
        return this;
    }
}
