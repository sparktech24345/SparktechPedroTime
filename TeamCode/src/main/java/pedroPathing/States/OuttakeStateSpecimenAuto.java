package pedroPathing.States;

import static pedroPathing.PositionStorage.isOuttakeStateSpecimen;
import static pedroPathing.PositionStorage.outakeArmServoPosition;
import static pedroPathing.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.PositionStorage.outtakeArmSpecimenPut;
import static pedroPathing.PositionStorage.stateStringOutake;
import static pedroPathing.PositionStorage.wasOuttakeStateSpecimen;
import static pedroPathing.PositionStorage.wasisOuttakeStateSpecimen;

// Define Outtake States
public class OuttakeStateSpecimenAuto implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSpecimen...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateSpecimenAuto";
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        wasisOuttakeStateSpecimen = true;
        outakeSampleServoPosition=0+6;
        //outakeArmServoPosition = 150; //190
        //if(armServoPos>=100)
            outakeArmServoPosition = outtakeArmSpecimenPut;
        outakeTargetPos = -outakeTargetPosAdder-1330;
    }
}
