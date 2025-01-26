package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;

import static pedroPathing.States.PositionStorage.isOuttakeStateStandbyWithSample;
import static pedroPathing.States.PositionStorage.outakeArmServoPosition;
import static pedroPathing.States.PositionStorage.outakeSampleRetracted;
import static pedroPathing.States.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.States.PositionStorage.outakeTargetPos;
import static pedroPathing.States.PositionStorage.outakeTransferPos;
import static pedroPathing.States.PositionStorage.servoextended;
import static pedroPathing.States.PositionStorage.spinyOutputToggle;
import static pedroPathing.States.PositionStorage.stateStringOutake;

public class OuttakeStateTranfer implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateStandbyDownWithSample...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateTranfer";
        isOuttakeStateStandbyWithSample = true;
        spinyOutputToggle = true;
        outakeArmServoPosition = 30;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
    }
}
