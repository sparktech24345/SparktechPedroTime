package pedroPathing.States;

import static pedroPathing.States.PositionStorage.IntakeServoColectPos;
import static pedroPathing.States.PositionStorage.PickyUppyOnce;
import static pedroPathing.States.PositionStorage.gravityAdder;
import static pedroPathing.States.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.States.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.States.PositionStorage.isIntakeStateExtended;
import static pedroPathing.States.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.States.PositionStorage.stateStringIntake;
import static pedroPathing.States.PositionStorage.wasActivePastActiveIntake;
import static pedroPathing.States.PositionStorage.wasActiveintake;
import static pedroPathing.States.PositionStorage.wasBambuExtended;
import static pedroPathing.States.PositionStorage.wasIntakeStateExtended;

public class IntakeStateExtendedRo2v2 implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateExtended...");
        // Set servo and motor positions for this state
        stateStringIntake = "IntakeStateExtended";
        isIntakeStateExtended = true;
        wasIntakeStateExtended = true;
        wasActiveintake = true;
        wasActivePastActiveIntake = true;
        wasBambuExtended = true;
        PickyUppyOnce = true;
        intakeRotateServoPosition = IntakeServoColectPos + pickUpAngleRo2V2Adder;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = 1;
    }
}
