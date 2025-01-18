package pedroPathing.States;

import static pedroPathing.States.PositionStorage.gravityAdder;
import static pedroPathing.States.PositionStorage.intakeActualZero;
import static pedroPathing.States.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.States.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.States.PositionStorage.intakeTargetPos;
import static pedroPathing.States.PositionStorage.intakeTransferAngles;
import static pedroPathing.States.PositionStorage.isIntakeStateRectracted;
import static pedroPathing.States.PositionStorage.stateStringIntake;
import static pedroPathing.States.PositionStorage.wasActivePastActiveIntake;

import pedroPathing.Toggle;

public class IntakeStateWallPURetraction implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "Intake state retracted for wall pickup";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = 0;
        intakeTargetPos = 0;
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
