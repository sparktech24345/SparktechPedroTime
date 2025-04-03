package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.IntakeServoColectPos;
import static pedroPathing.newOld.PositionStorage.gravityAdder;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;

public class IntakeStateExtendedHM implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateExtended...");
        // Set servo and motor positions for this state
        stateStringIntake = "IntakeStateExtended HUMAN PLAYER SAMPLE OUT";
        intakeRotateServoPosition = IntakeServoColectPos + pickUpAngleRo2V2Adder;
        if(gravityAdder==0)
            gravityAdder = 7;
    }
}
