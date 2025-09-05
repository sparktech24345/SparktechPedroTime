package Experimental.HelperClasses.Actions;

import static Experimental.HelperClasses.GlobalStorage.telemetryInstance;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class DelayAction extends Action {
    private ElapsedTime timer;
    private double waitTimeMS = 0;
    private double delay = 0;

    public DelayAction(boolean waitForPrevious, double milliseconds) {
        super(waitForPrevious);
        this.waitTimeMS = milliseconds;
        timer = new ElapsedTime();
        timer.reset();
        this.DoneCondition = () -> timer.milliseconds() >= waitTimeMS + delay;
    }

    public DelayAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }

    @Override
    public void update(boolean isPreviousDone) {
        if (!start) {
            start = ExecutionCondition.getAsBoolean() && (!waitForPrevious || isPreviousDone);
            delay = timer.milliseconds();
        }
        if (!done && start) {
            Execution.run();
            done = DoneCondition.getAsBoolean();
        }
    }
}
