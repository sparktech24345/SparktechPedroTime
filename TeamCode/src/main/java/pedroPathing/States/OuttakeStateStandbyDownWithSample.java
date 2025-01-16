package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;
public class OuttakeStateStandbyDownWithSample implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateStandbyDownWithSample...");
        // Set servo and motor positions for this state
        stateStringOutake = "Outtake State Standby Down With Sample";
        isOuttakeStateStandbyWithSample = true;
        spinyOutputToggle = true;
        outakeArmServoPosition = outakeTransferPos;
        outakeTargetPos =0;
        outakeSampleServoPosition = outakeSampleRetracted;
    }
}
