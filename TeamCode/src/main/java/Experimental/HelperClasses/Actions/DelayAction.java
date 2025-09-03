package Experimental.HelperClasses.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayAction extends Action {

    private ElapsedTime timer;
    private double waitTimeMS = 0;

    public DelayAction(boolean waitForPrevious, double milliseconds) {
        this.waitForPrevious = waitForPrevious;
        this.waitTimeMS = milliseconds;
        timer = new ElapsedTime();
        this.DoneCondition = () -> timer.milliseconds() >= waitTimeMS;
    }
}
