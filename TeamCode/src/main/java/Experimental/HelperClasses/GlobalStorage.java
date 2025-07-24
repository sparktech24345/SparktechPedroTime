package Experimental.HelperClasses;

import Experimental.HelperClasses.RobotState;

import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;

public class GlobalStorage {

    // OTHER STUFF
    public static OpMode currentOpMode;
    public static boolean waitForIntakeExtSet = false;
    public static boolean shouldContinue = false;
    public static boolean followerShouldContinue = false;


    // STATES

    public static RobotState currentRobotState = RobotState.StartState;

    public static IntakePosition currentIntakePos = currentRobotState.intakePosition;
    public static IntakeExtension currentIntakeExt = currentRobotState.intakeExtension;
    public static OuttakeExtension currentOuttakeExt = currentRobotState.outtakeExtension;
    public static OuttakeArmPosition currentOuttakeArmPos = currentRobotState.outtakeArmPosition;
    public static OuttakeClawPosition currentOuttakeClawPos = currentRobotState.outtakeClawPosition;


    public static RobotState targetRobotState = RobotState.StartState;

    public static IntakePosition targetIntakePos = targetRobotState.intakePosition;
    public static IntakeExtension targetIntakeExt = targetRobotState.intakeExtension;
    public static OuttakeExtension targetOuttakeExt = targetRobotState.outtakeExtension;
    public static OuttakeArmPosition targetOuttakeArmPos = targetRobotState.outtakeArmPosition;
    public static OuttakeClawPosition targetOuttakeClawPos = targetRobotState.outtakeClawPosition;
}
