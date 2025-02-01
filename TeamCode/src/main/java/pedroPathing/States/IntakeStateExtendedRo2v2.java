package pedroPathing.States;

import static pedroPathing.PositionStorage.*;

public class IntakeStateExtendedRo2v2 implements State {
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
        PickyUppyOnce = true;
        intakeRotateServoPosition = IntakeServoColectPos + pickUpAngleRo2V2Adder;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = 1;
    }
}
