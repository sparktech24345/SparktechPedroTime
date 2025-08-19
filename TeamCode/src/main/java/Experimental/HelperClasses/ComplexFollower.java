package Experimental.HelperClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import static Experimental.HelperClasses.GlobalStorage.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import static Experimental.StatesAndPositions.AutoOfSpecStatesAndPos.*;

public class ComplexFollower {
    private boolean shouldContinue = true;
    public boolean isDone = true;
    public Follower follower;
    public double currentX;
    public double currentY;
    public double currentHeading;
    private Pose currentTargetPos;
    private Pose currentPos;
    private Path pathToFollow;
    private Queue<Pose> poseQueue = new ArrayDeque<>();

    ComplexFollower(Follower follower) {
        Constants.setConstants(F_Constants, L_Constants);
        this.follower = follower;
        this.follower.update();
        currentPos = startPose;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
    }
    ComplexFollower(HardwareMap hardwareMap) {
        Constants.setConstants(F_Constants, L_Constants);
        this.follower = new Follower(hardwareMap, F_Constants, L_Constants);
        follower.update();
        currentPos = startPose;
        currentX = currentPos.getX();
        currentY = currentPos.getY();
        currentHeading = currentPos.getHeading();
    }

    public void follow(Pose targetPos) {
        poseQueue.add(targetPos);
    }

    public void Continue() {
        shouldContinue = true;
    }

    public void update() {
        if (currentOpMode == OpMode.Autonomous) {
            follower.update();
            if (follower.isBusy()) {
                isDone = false;
                currentPos = follower.getPose();
                currentX = currentPos.getX();
                currentY = currentPos.getY();
                currentHeading = currentPos.getHeading();
            }
            else if (shouldContinue && !poseQueue.isEmpty()) {
                currentTargetPos = poseQueue.poll();
                pathToFollow = new Path(new BezierLine(currentPos, currentTargetPos));
                follower.followPath(pathToFollow);
                shouldContinue = false;
                follower.update();
            }
            else isDone = true;
        }
    }

    public void stopAll() {
        interrupt();
        poseQueue.clear();
    }

    public void interrupt() {
        if (follower.isBusy()) follower.breakFollowing();
    }

    public void checkNotMoving() {
//        if (follower.isBusy() &&)
    }

    public void telemetry() {
        telemetryInstance.addData("Follower current X", currentX);
        telemetryInstance.addData("Follower current Y", currentY);
        telemetryInstance.addData("Follower current Heading", currentHeading);
        telemetryInstance.addData("Follower is busy", follower.isBusy());
    }
}
