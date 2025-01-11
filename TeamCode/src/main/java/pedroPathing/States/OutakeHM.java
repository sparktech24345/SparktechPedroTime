package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;
// Define Outtake States
public class OutakeHM implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateSpecimen...");
        // Set servo and motor positions for this state
        stateStringOutake = "OutakeHM";
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        wasisOuttakeStateSpecimen = true;
        if(outakeSampleServoPosition != servoextended)
            outakeSampleServoPosition=outakeSampleRetracted;
        outakeArmServoPosition = 345;
    }
}
