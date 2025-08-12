package Experimental.HelperClasses.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayAction extends Action {

    private ElapsedTime timer;
    private double waitTimeMS = 0;

    DelayAction(boolean waitForPrevious, double milliseconds) {
        this.waitForPrevious = waitForPrevious;
        this.waitTimeMS = milliseconds;
    }

    @Override
    public void execute() {
        timer = new ElapsedTime();
        started = true;
    }

    @Override
    public void update() {
        done = timer.milliseconds() >= waitTimeMS;
    }
}
