package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;

public class OuttakeStateStandbyWithSample implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateStandbyDownWithSample...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateStandbyWithSample";
        isOuttakeStateStandbyWithSample = true;
        outakeSampleServoPosition = outakeSampleRetracted;
        outakeArmServoPosition = outakeTransferPos;
        outakeTargetPos =0;
    }
}