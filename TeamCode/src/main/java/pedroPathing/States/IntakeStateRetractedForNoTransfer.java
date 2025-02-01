package pedroPathing.States;

import static pedroPathing.PositionStorage.gravityAdder;
import static pedroPathing.PositionStorage.intakeActualZero;
import static pedroPathing.PositionStorage.intakeExtraSpinDoOnce;
import static pedroPathing.PositionStorage.intakeExtraSpinTimer;
import static pedroPathing.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.intakeSlidersRo2Transfer;
import static pedroPathing.PositionStorage.intakeTargetPos;
import static pedroPathing.PositionStorage.intakeTransferAngles;
import static pedroPathing.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.PositionStorage.stateStringIntake;
import static pedroPathing.PositionStorage.transferDisabled;
import static pedroPathing.PositionStorage.wasActivePastActiveIntake;

import pedroPathing.Toggle;

public class IntakeStateRetractedForNoTransfer implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeStateRetracted";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = 200;
        intakeTargetPos = intakeSlidersRo2Transfer;
        intakeMotorPickUpPower = 0;
    }
}
