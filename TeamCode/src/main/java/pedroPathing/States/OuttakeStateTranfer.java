package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.*;

import static pedroPathing.newOld.PositionStorage.isOuttakeStateStandbyWithSample;
import static pedroPathing.newOld.PositionStorage.outakeArmServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeTargetPos;
import static pedroPathing.newOld.PositionStorage.servoextended;
import static pedroPathing.newOld.PositionStorage.spinyOutputToggle;
import static pedroPathing.newOld.PositionStorage.stateStringOutake;

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
