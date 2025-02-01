package pedroPathing.States;

import static pedroPathing.PositionStorage.*;

import static pedroPathing.PositionStorage.isOuttakeStateStandbyWithSample;
import static pedroPathing.PositionStorage.outakeArmServoPosition;
import static pedroPathing.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.servoextended;
import static pedroPathing.PositionStorage.spinyOutputToggle;
import static pedroPathing.PositionStorage.stateStringOutake;

public class OuttakeStateTranfer implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateStandbyDownWithSample...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateTranfer";
        isOuttakeStateStandbyWithSample = true;
        spinyOutputToggle = true;
        outakeArmServoPosition = outtakeArmServoPosAtRo2v2TransferPickUp;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
    }
}
