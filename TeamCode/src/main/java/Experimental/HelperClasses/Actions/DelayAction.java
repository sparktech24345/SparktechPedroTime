package Experimental.HelperClasses.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class DelayAction extends Action {
    private final ElapsedTime timer;
    private double waitTimeMS = 0;

    public DelayAction(boolean waitForPrevious, double milliseconds) {
        this.waitForPrevious = waitForPrevious;
        this.waitTimeMS = milliseconds;
        timer = new ElapsedTime();
        this.DoneCondition = () -> timer.milliseconds() >= waitTimeMS;
    }

    public DelayAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }
}
