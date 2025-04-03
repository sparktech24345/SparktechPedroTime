package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.gravityAdder;
import static pedroPathing.newOld.PositionStorage.intakeRotateForWallPickUp;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;

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
