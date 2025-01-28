package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;
// Define Outtake States
public class OuttakeStateSpecimen implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSpecimen...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateSpecimen";
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        wasisOuttakeStateSpecimen = true;
        outakeSampleServoPosition=0+6;
        //outakeArmServoPosition = 150; //190
        //if(armServoPos>=100)
            outakeArmServoPosition = outtakeArmSpecimenPut;
        outakeTargetPos = -outakeTargetPosAdder-1350;
    }
}
