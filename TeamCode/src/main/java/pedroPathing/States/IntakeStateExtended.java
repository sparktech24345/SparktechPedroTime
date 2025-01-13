package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;

public class IntakeStateExtended implements State {
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
        intakeRotateServoPosition = IntakeServoColectPos;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = 1;
    }
}
