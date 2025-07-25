package Experimental.HelperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;

public class GlobalStorage {

    public static double eval(boolean bool) {
        return (bool ? 1 : 0);
    }

    public static void resetTimers() {
        intakeExtTimer.reset();
        intakePosTimer.reset();
        outtakeExtTimer.reset();
        outtakeArmPosTimer.reset();
        outtakeClawPosTimer.reset();
    }

    // HARDWARE TIMERS

    public static ElapsedTime intakeExtTimer = new ElapsedTime();
    public static ElapsedTime intakePosTimer = new ElapsedTime();
    public static ElapsedTime outtakeExtTimer = new ElapsedTime();
    public static ElapsedTime outtakeArmPosTimer = new ElapsedTime();
    public static ElapsedTime outtakeClawPosTimer = new ElapsedTime();


    // TIME TO MOVE TO POS IN MILLISECONDS

    public static double intakeExtMoveTime = 300;
    public static double intakePosMoveTime = 200;
    public static double outtakeExtMoveTime = 200;
    public static double outtakeArmPosMoveTime = 250;
    public static double outtakeClawPosMoveTime = 250;

    // HARDWARE NAMES

    public static String intakeSpinName         = "intakespin";
    public static String intakeExtendName       = "intakemotor";
    public static String intakePosName          = "intakeRotateServo";
    public static String outtakeExtendLeftName  = "outakeleftmotor";
    public static String outtakeExtendRightName = "outakerightmotor";
    public static String outtakeArmName         = "outakeArmServo";
    public static String outtakeClawName        = "outakeSampleServo";
    public static String frontRightName         = "frontright";
    public static String frontLeftName          = "frontleft";
    public static String backRightName          = "backright";
    public static String backLeftName           = "backleft";
    public static String colorSensorName        = "sensorColor";

    // OTHER STUFF
    public static ColorSet currentTeam = null;
    public static OpMode currentOpMode;
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
