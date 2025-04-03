package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.*;

import pedroPathing.newOld.Toggle;

public class IntakeStateRetracted implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeStateRetracted";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = intakeTransferAngles;
        intakeTargetPos = intakeActualZero; //Waat is going on here why is angle for motor please check
        //intakeMotorPickUpPower = 0;
        if(!transferDisabled) {
            intakeExtraSpinTimer = System.currentTimeMillis();
            intakeExtraSpinDoOnce = true;
            intakeMotorPickUpPower = 0.7;
        } else intakeMotorPickUpPower = 0;
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
