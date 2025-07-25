package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import Experimental.StatesAndPositions.IntakeExtension;

public class StateQueuer {

    private RobotState loadedState = RobotState.StartState;
    public void setState(RobotState state) {
        loadedState = state;
    }

    public void loadState() {
        currentRobotState = loadedState;
        if (currentRobotState.intakeExtension != IntakeExtension.IGNORE)
            currentIntakeExt = currentRobotState.intakeExtension;
        currentIntakePos = currentRobotState.intakePosition;
        currentOuttakeExt = currentRobotState.outtakeExtension;
        currentOuttakeArmPos = currentRobotState.outtakeArmPosition;
        currentOuttakeClawPos = currentRobotState.outtakeClawPosition;
    }

}
