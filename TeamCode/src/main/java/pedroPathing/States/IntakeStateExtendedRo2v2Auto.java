package pedroPathing.States;

import static pedroPathing.PositionStorage.IntakeServoColectPos;
import static pedroPathing.PositionStorage.PickyUppyOnce;
import static pedroPathing.PositionStorage.gravityAdder;
import static pedroPathing.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.isIntakeStateExtended;
import static pedroPathing.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.PositionStorage.stateStringIntake;
import static pedroPathing.PositionStorage.wasActivePastActiveIntake;
import static pedroPathing.PositionStorage.wasActiveintake;
import static pedroPathing.PositionStorage.wasBambuExtended;
import static pedroPathing.PositionStorage.wasIntakeStateExtended;

public class IntakeStateExtendedRo2v2Auto implements State {
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
        intakeRotateServoPosition = IntakeServoColectPos + pickUpAngleRo2V2Adder - 65;
        intakeMotorPickUpPower = 1;
    }
}
