package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.isOuttakeStateSpecimen;
import static pedroPathing.newOld.PositionStorage.outakeArmServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeTargetPos;
import static pedroPathing.newOld.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.outtakeArmSpecimenPut;
import static pedroPathing.newOld.PositionStorage.stateStringOutake;
import static pedroPathing.newOld.PositionStorage.wasOuttakeStateSpecimen;
import static pedroPathing.newOld.PositionStorage.wasisOuttakeStateSpecimen;

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
