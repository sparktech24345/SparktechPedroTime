package Experimental.HelperClasses.Actions;

public abstract class Action {
    public boolean waitForPrevious;

    public boolean started = false;

    public boolean done = false;

    public Action() {
        this.waitForPrevious = true;
    }
    public Action(boolean waitForPrevious) {
        this.waitForPrevious = waitForPrevious;
    }

    public abstract void execute();

    public abstract void update();
}
