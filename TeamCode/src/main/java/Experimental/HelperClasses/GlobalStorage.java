package Experimental.HelperClasses;

import android.util.Pair;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Experimental.StatesAndPositions.ColorSet;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class GlobalStorage {

    public static double clamp(double val, double limit) {
        return Math.max(-limit, Math.min(limit, val));
    }

    public static double eval(boolean val) {
        return (val ? 1 : 0);
    }
    public static boolean eval(double val) {
        return val != 0;
    }
    public static <Tx, Ty> Pair<Tx, Ty> make_pair(Tx arg1, Ty arg2) { return new Pair<>(arg1, arg2); }

    // INSTANCES

    public static ComplexFollower followerInstance = null;
    public static ComplexGamepad gamepadInstance = null;
    public static HardwareMap hardwareMapInstance = null;
    public static Telemetry telemetryInstance = null;
    public static StateQueuer queuerInstance = null;
    public static RobotController robotControllerInstance = null;


    // CONSTANTS

    public static Class<?> F_Constants = FConstants.class;
    public static Class<?> L_Constants = LConstants.class;


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

    // OTHER S(TUFF)
    public static ColorSet currentTeam = ColorSet.Undefined;
    public static OpModes currentOpModes = OpModes.TeleOP;
}
