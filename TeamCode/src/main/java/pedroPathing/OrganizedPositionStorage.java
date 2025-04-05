package pedroPathing;

@com.acmerobotics.dashboard.config.Config
public class OrganizedPositionStorage {
    // MISC
    public static double PIDincrement;
    public static double gravityAdder;

    // chassis / sasiu
    public static double chassisBackLeftPow = 0;
    public static double chassisBackRightPow = 0;
    public static double chassisFrontLeftPow = 0;
    public static double chassisFrontRightPow = 0;

    // intake
    public static double intakePivotServoPos;
    public static double intakeSpinMotorPow;
    public static double intakeExtendMotorTargetPos;
    public static double intakeExtendMotorPow;

    public static double intakeTargetPosAdder;

    // outtake
    public static double outtakePivotServoPos;
    public static double outtakeClawServoPos;
    public static double outtakeExtendMotorTargetPos;
    public static double outtakeExtendMotorPow;

    public static double outtakeTargetPosAdder;






    // CONSTANTE*


    //Outtake

    //outtake claw
    public static double outtakeClawServoExtendedPos = 150;
    public static double outtakeClawServoRetractedPos = 12;

    // outtake pivot
    public static double outtakePivotServoWallPickupPos = 305;
    public static double outtakePivotServoHighRungHangPos = 183;
    public static double outtakePivotServoBasketPos = 50;
    public static double outtakePivotServoTransferPos = 150; //NOT SET


    // outtake sliders

    public static double outtakeMotorMaxPos = 2038;
    public static double outtakeMotorActualZeroPos = 0;



    //intake

    //intake pivot
    public static double intakePivotServoPickupPos = 92;
    public static double intakePivotServoOutputTruBotPos = 15;
    public static double intakePivotServoTransferPos = 35; // NOT SET



    // BOOLEANS

    //misc
    public static boolean isYellowSampleNotGood = false;


    //is pressed
    public static boolean isPressedA1 = false;
    public static boolean isPressedA2 = false;
    public static boolean isPressedB1 = false;
    public static boolean isPressedB2 = false;
    public static boolean isPressedX1 = false;
    public static boolean isPressedX2 = false;
    public static boolean isPressedY1 = false;
    public static boolean isPressedY2 = false;
    public static boolean isPressedDL1 = false;


    //intake stuff
    public static boolean isAfterIntakeBeenDownColecting = false;
    public static boolean isIntakeOutputting = false;
    public static boolean isAfterBotHasBeenOutputting = false;


    //outtake stuff
    public static boolean isAfterOuttakeScoredSpecimen = false;
    public static boolean isAfterOuttakeScoredBasketSample = false;
    public static boolean isAfterOuttakeClosedClawAtWallSpecimen = false;





    //LONGS / TIMERS

    //outtake stuff
    public static long outtakeSpecimenAfterScoreTimer;
    public static long outtakeAfterBasketSampleScoreTimer;
    public static long outtakeAfterHasClosedClawAtWallSpecimenTimer;
    public static long intakeOutputtingTimer;


    public static void resetStuff() {

    }
}