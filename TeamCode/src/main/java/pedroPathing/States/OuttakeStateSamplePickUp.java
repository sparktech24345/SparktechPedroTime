package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.*;
public class OuttakeStateSamplePickUp implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSamplePickUp...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateSamplePickUp";
        isOuttakeStateSamplePickUp = true;
        outakeArmServoPosition = outakeTransferPos;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
    }
}
