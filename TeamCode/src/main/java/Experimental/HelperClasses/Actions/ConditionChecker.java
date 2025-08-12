package Experimental.HelperClasses.Actions;

import Experimental.HelperClasses.Checkable;

public class ConditionChecker extends Action {

    private Checkable executor;

    public ConditionChecker(Checkable executor) {
        this.executor = executor;
    }

    @Override
    public void execute() {
        started = true;
    }

    @Override
    public void update() {
        done = executor.check();
    }
}
