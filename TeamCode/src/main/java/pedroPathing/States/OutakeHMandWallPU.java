package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.*;
// Define Outtake States
public class OutakeHMandWallPU implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSpecimen...");
        // Set servo and motor positions for this state
        stateStringOutake = "OutakeHMandWallPU";
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        wasisOuttakeStateSpecimen = true;
        outakeSampleServoPosition=servoextended;
        outakeArmServoPosition = OuttakeArmWallPickUpPosition;
        outakeTargetPos =0;
    }
}
