package pedroPathing.States;

import static pedroPathing.newOld.PositionStorage.intakeOutputTruBotPosition;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;

public class IntakeWaitForOutputTruBot implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateExtended...");
        // Set servo and motor positions for this state
        stateStringIntake = "IntakeWaitForOutputTruBot";
        intakeRotateServoPosition = intakeOutputTruBotPosition;
    }
}
