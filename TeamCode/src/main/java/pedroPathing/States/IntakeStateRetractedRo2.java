package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;


import pedroPathing.Toggle;

public class IntakeStateRetractedRo2 implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeStateRetracted";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = intakeRo2SmashPos;
        intakeTargetPos = intakeSlidersRo2Transfer; //Waat is going on here why is angle for motor please check
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
