package pedroPathing.States;

import static pedroPathing.PositionStorage.IntakeServoColectPos;
import static pedroPathing.PositionStorage.gravityAdder;
import static pedroPathing.PositionStorage.intakeOutputTruBotPosition;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.pickUpAngleRo2V2Adder;
import static pedroPathing.PositionStorage.stateStringIntake;

public class IntakeWaitForOutputTruBot implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateExtended...");
        // Set servo and motor positions for this state
        stateStringIntake = "IntakeWaitForOutputTruBot";
        intakeRotateServoPosition = intakeOutputTruBotPosition;
    }
}
