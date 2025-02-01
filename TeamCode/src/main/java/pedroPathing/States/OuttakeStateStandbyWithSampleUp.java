package pedroPathing.States;

import static pedroPathing.PositionStorage.*;

public class OuttakeStateStandbyWithSampleUp implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateStandbyDownWithSample...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateStandbyWithSampleUp";
        isOuttakeStateStandbyWithSample = true;
        spinyOutputToggle = true;
        outakeArmServoPosition = outakeTransferPos+35;
        outakeTargetPos =0;
    }
}