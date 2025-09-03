package Experimental.HelperClasses.Actions;

import java.util.function.BooleanSupplier;

public abstract class Action {
    protected boolean waitForPrevious;
    protected boolean start = false;
    protected boolean done = false;

    protected BooleanSupplier ExecutionCondition = () -> true;
    protected BooleanSupplier DoneCondition = () -> true;

    protected Runnable Execution = () -> {};

    public Action() {
        this.waitForPrevious = true;
    }
    public Action(boolean waitForPrevious) {
        this.waitForPrevious = waitForPrevious;
    }

    public boolean waits() {
        return waitForPrevious;
    }

    public boolean finished() {
        return done;
    }

    public boolean started() { return start; }

    public void update(boolean isPreviousDone) {
        if (!start) {
           start = ExecutionCondition.getAsBoolean() && (!waitForPrevious || isPreviousDone);
        }
        else {
            Execution.run();
            done = DoneCondition.getAsBoolean();
        }
    }
}
