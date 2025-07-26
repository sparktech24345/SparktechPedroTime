package pedroPathing.customUtilities.autoRecorder;

import java.util.ArrayList;
import java.util.List;

public class PoseSequence {
    public ArrayList<PoseData> poseSequence;

    public PoseSequence(ArrayList<PoseData> poseSequence) {
        this.poseSequence = poseSequence;
    }

    public PoseSequence() {
        this.poseSequence = new ArrayList<>();
    }

    public void addPose(PoseData pose) {
        PoseData copy = new PoseData(pose.getX(), pose.getY(), pose.getHeading());
        poseSequence.add(copy);
    }

    public List<PoseData> getPoseSequence() {return poseSequence;}
    public void setPoseSequence(ArrayList<PoseData> poseSequence) {this.poseSequence = poseSequence;}
}
