package pedroPathing.States;

import static pedroPathing.PositionStorage.IntakeWallPickUpPosition;
import static pedroPathing.PositionStorage.gravityAdder;
import static pedroPathing.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.PositionStorage.intakeRotateForWallPickUp;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.intakeTargetPos;
import static pedroPathing.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.PositionStorage.stateStringIntake;
import static pedroPathing.PositionStorage.wasActivePastActiveIntake;

import pedroPathing.Toggle;

public class IntakeStateWallPURetractionHM implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "Intake state retracted for wall pickup HUMAN PLAYER";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = intakeRotateForWallPickUp;
    }
}
