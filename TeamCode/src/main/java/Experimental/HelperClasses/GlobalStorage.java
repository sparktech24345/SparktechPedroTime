package Experimental.HelperClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class GlobalStorage {

    public static double eval(boolean val) {
        return (val ? 1 : 0);
    }
    public static boolean eval(double val) {
        return val != 0;
    }

//    public static void resetTimers() {
//        intakeExtTimer.reset();
//        intakePosTimer.reset();
//        outtakeExtTimer.reset();
//        outtakeArmPosTimer.reset();
//        outtakeClawPosTimer.reset();
//    }


    // INSTANCES

    public static ComplexFollower followerInstance = null;
    public static ComplexGamepad gamepadInstance = null;
    public static HardwareMap hardwareMapInstance = null;
    public static MultipleTelemetry telemetryInstance = null;
    public static DriveTrain driveTrainInstance = null;
    public static Intake intakeInstance = null;
    public static Outtake outtakeInstance = null;
    public static StateQueuer queuerInstance = null;


    // CONSTANTS

    public static Class<?> F_Constants = FConstants.class;
    public static Class<?> L_Constants = LConstants.class;


    // HARDWARE TIMERS

//    public static ElapsedTime intakeExtTimer = new ElapsedTime();
//    public static ElapsedTime intakePosTimer = new ElapsedTime();
//    public static ElapsedTime outtakeExtTimer = new ElapsedTime();
//    public static ElapsedTime outtakeArmPosTimer = new ElapsedTime();
//    public static ElapsedTime outtakeClawPosTimer = new ElapsedTime();


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
    public static ColorSet currentTeam = ColorSet.Black;
    public static OpMode currentOpMode = OpMode.TeleOP;
    public static boolean shouldContinue = false;
    public static boolean followerShouldContinue = false;


    // STATES

    public static void loadRobotState() {
        currentIntakeExt = currentRobotState.intakeExtension;
        currentIntakePos = currentRobotState.intakePosition;
        currentOuttakeExt = currentRobotState.outtakeExtension;
        currentOuttakeArmPos = currentRobotState.outtakeArmPosition;
        currentOuttakeClawPos = currentRobotState.outtakeClawPosition;
    }

    public static RobotState currentRobotState = RobotState.StartState;

    public static IntakePosition currentIntakePos = currentRobotState.intakePosition;
    public static IntakeExtension currentIntakeExt = currentRobotState.intakeExtension;
    public static OuttakeExtension currentOuttakeExt = currentRobotState.outtakeExtension;
    public static OuttakeArmPosition currentOuttakeArmPos = currentRobotState.outtakeArmPosition;
    public static OuttakeClawPosition currentOuttakeClawPos = currentRobotState.outtakeClawPosition;
}
