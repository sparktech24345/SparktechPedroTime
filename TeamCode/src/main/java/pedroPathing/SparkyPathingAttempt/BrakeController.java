package pedroPathing.SparkyPathingAttempt;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

public class BrakeController {
    double breakingThreshold = 12;

    /** Stuff this needs to do
     * update method: take the current position,do the math, if need be correct follower, if not do not
     * math method
     * checker method
     * output method
     * all in main method
     */

    Follower pedroFollower;
    public BrakeController(Follower follower) {
        this.pedroFollower = follower;
    }
    public void breakControllerUpdate(Pose targetPose){

        Pose currentPose = pedroFollower.getPose();
        boolean isInPositionToBreak = shouldBrake(currentPose,targetPose,breakingThreshold);






    }
    public boolean shouldBrake(Pose currentPose, Pose targetPose, double brakingThreshold) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        return Math.hypot(dx, dy) < brakingThreshold;
    }

}
