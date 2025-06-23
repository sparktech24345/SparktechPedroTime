package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.*;

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
        intakeRotateServoPosition = intakeRotateForWallPickUp;
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
