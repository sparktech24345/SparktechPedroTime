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

public class IntakeStateWallPURetractionRo2v2 implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeStateWallPURetractionRo2v2";
        isIntakeStateRectracted = true;
        intakeTargetPos = IntakeWallPickUpPosition;
        intakeMotorPickUpPower = 0;
        if(wasActivePastActiveIntake){
            wasActivePastActiveIntake = false;
            Toggle.toggledUp = false;
            Toggle.toggle_varUp = false;
            Toggle.toggledLeft = false;
            Toggle.toggle_varLeft = false;
            Toggle.toggledDown = false;
            Toggle.toggle_varDown = false;
            Toggle.toggledRight = false;
            Toggle.toggle_varRight = false;
        }
    }
}
