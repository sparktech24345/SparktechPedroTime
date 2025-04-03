package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.isOuttakeStateSpecimen;
import static pedroPathing.newOld.PositionStorage.outakeArmServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeSampleRetracted;
import static pedroPathing.newOld.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeTargetPos;
import static pedroPathing.newOld.PositionStorage.outtakeArmSpecimenPut;
import static pedroPathing.newOld.PositionStorage.servoextended;
import static pedroPathing.newOld.PositionStorage.stateStringOutake;
import static pedroPathing.newOld.PositionStorage.wasOuttakeStateSpecimen;
import static pedroPathing.newOld.PositionStorage.wasisOuttakeStateSpecimen;

// Define Outtake States
public class OuttakeSpecimenHangAuto implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSpecimen...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeSpecimenHangAuto";
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        wasisOuttakeStateSpecimen = true;
        if(outakeSampleServoPosition != servoextended)
            outakeSampleServoPosition=outakeSampleRetracted;
        outakeArmServoPosition = outtakeArmSpecimenPut;
        outakeTargetPos = -1880;
    }
}
