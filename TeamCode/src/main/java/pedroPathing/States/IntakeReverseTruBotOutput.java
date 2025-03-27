package pedroPathing.States;

import static pedroPathing.PositionStorage.*;

public class IntakeReverseTruBotOutput implements State {
    @Override
    public void execute() {
        //System.out.println("Executing IntakeStateRetracted...");
        // Set servo and motor positions for this state
        //telemetry.addData("IntakeStateRectracted",true);
        gravityAdder = 0;
        stateStringIntake = "IntakeReverseTruBotOutput";
        isIntakeStateRectracted = true;
        intakeRotateServoPosition = intakeOutputTruBotPosition;
    }
}
