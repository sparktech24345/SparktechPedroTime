package pedroPathing.States;

import static pedroPathing.PositionStorage.isOuttakeStateSpecimen;
import static pedroPathing.PositionStorage.outakeArmServoPosition;
import static pedroPathing.PositionStorage.outakeSampleRetracted;
import static pedroPathing.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.outtakeArmSpecimenPut;
import static pedroPathing.PositionStorage.servoextended;
import static pedroPathing.PositionStorage.stateStringOutake;
import static pedroPathing.PositionStorage.wasOuttakeStateSpecimen;
import static pedroPathing.PositionStorage.wasisOuttakeStateSpecimen;

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
