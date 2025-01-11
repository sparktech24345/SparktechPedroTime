package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;
public class OuttakeStateSamplePickUp implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSamplePickUp...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateSamplePickUp";
        isOuttakeStateSamplePickUp = true;
        outakeArmServoPosition = 60;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
    }
}
