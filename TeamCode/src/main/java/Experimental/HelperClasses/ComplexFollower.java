package Experimental.HelperClasses;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import static Experimental.HelperClasses.GlobalStorage.*;

public class ComplexFollower {
    public Follower follower;
    private double currentX;
    private double currentY;
    private Pose currentTargetPos;
    private Pose currentPos;
    private Path pathToFollow;
    private Queue<Pose> poseQueue = new ArrayDeque<>();

    ComplexFollower(Follower follower) {
        this.follower = follower;
        currentPos = follower.getPose();
        currentX = currentPos.getX();
        currentX = currentPos.getY();
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (currentOpMode == OpMode.Autonomous) {
                    if (follower.isBusy()) {
                        currentPos = follower.getPose();
                        currentX = currentPos.getX();
                        currentY = currentPos.getY();
                        continue;
                    }
                    if (poseQueue.isEmpty())
                        continue;
                    if (followerShouldContinue) {
                        pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
                        follower.followPath(pathToFollow);
                    }
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        });
    }

    public void follow(Pose targetPos) {
        poseQueue.add(targetPos);
    }

    public void stopAll() {
        poseQueue = new ArrayDeque<>();
        interrupt();
    }

    public void interrupt() {
        if (follower.isBusy()) follower.breakFollowing();
    }
}
