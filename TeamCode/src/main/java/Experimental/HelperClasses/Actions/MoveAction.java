package Experimental.HelperClasses.Actions;

import com.pedropathing.localization.Pose;

import static Experimental.HelperClasses.GlobalStorage.*;

import java.util.function.BooleanSupplier;

public class MoveAction extends Action {

    private boolean reachTarget = true;
    private final Pose moveTargetPos;

    public MoveAction(boolean waitForPrevious, Pose moveTargetPos, boolean waitToReachTarget) {
        this.waitForPrevious = waitForPrevious;
        this.moveTargetPos = moveTargetPos;
        this.reachTarget = waitToReachTarget;

        if (waitToReachTarget) this.DoneCondition = () -> followerInstance.getInstance().isBusy();
        this.Execution = () -> {
            followerInstance.follow(moveTargetPos);
            followerInstance.Continue();
        };
    }

    public MoveAction setExecutionCondition(BooleanSupplier exec) {
        this.ExecutionCondition = exec;
        return this;
    }
}
