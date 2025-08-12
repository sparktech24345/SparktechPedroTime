package Experimental.HelperClasses.Actions;

import com.pedropathing.localization.Pose;

import Experimental.HelperClasses.ComplexFollower;
import Experimental.HelperClasses.GlobalStorage;

public class MoveAction extends Action {
    private boolean reachTarget = true;
    private final ComplexFollower followerInstance = GlobalStorage.followerInstance;
    private final Pose moveTargetPos;
    public MoveAction(boolean waitForPrevious, Pose moveTargetPos, boolean waitToReachTarget) {
        this.waitForPrevious = waitForPrevious;
        this.moveTargetPos = moveTargetPos;
        this.reachTarget = waitToReachTarget;
    }

    @Override
    public void execute() {
        followerInstance.follow(moveTargetPos);
        followerInstance.Continue();
        started = true;
        if (!reachTarget) done = true;
    }

    @Override
    public void update() {
        done = !followerInstance.follower.isBusy();
    }
}
