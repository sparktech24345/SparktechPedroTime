package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.gravityAdder;
import static pedroPathing.newOld.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.intakeSlidersRo2Transfer;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;

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
