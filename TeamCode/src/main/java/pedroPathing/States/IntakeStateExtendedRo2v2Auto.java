package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.IntakeServoColectPos;
import static pedroPathing.newOld.PositionStorage.PickyUppyOnce;
import static pedroPathing.newOld.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.isIntakeStateExtended;
import static pedroPathing.newOld.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;
import static pedroPathing.newOld.PositionStorage.wasActivePastActiveIntake;
import static pedroPathing.newOld.PositionStorage.wasActiveintake;
import static pedroPathing.newOld.PositionStorage.wasBambuExtended;
import static pedroPathing.newOld.PositionStorage.wasIntakeStateExtended;

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
