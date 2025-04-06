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
    public static double intakeExtendMotorTargetPos=0;

    public static double intakeTargetPosAdder;

    // outtake
    public static double outtakePivotServoPos;
    public static double outtakeClawServoPos;
    public static double outtakeExtendMotorTargetPos=0;

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
    public static double outtakePivotServoTransferPos = 210;
    public static double outtakePivotServoStandByPos = 170;


    // outtake sliders

    public static double outtakeMotorMaxPos = 2038;
    public static double outtakeSliderSpecimenHangPos = 1100;
    public static double outtakeSlidersWallPickPos = 680;
    public static double outtakeMotorActualZeroPos = 0;



    //intake

    //intake pivot
    public static double intakePivotServoPickupPos = 210;
    public static double intakePivotServoOutputTruBotPos = 15;
    public static double intakePivotServoTransferPos = 140;



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
    public static boolean isAfterOuttakeClawClosedAfterTransfer = false;
    public static boolean isIntakeOutputtingManual = false;
    public static boolean isIntakeSpinMOtorAfterJustTaking = false;


    //outtake stuff
    public static boolean isAfterOuttakeScoredSpecimen = false;
    public static boolean isAfterOuttakeScoredBasketSample = false;
    public static boolean isAfterOuttakeClosedClawAtWallSpecimen = false;
    public static boolean isAtStateOfLettingBasketSampleGo = false;
    public static boolean isInNeedToGoToSpecimenTransferPos = false;





    //LONGS / TIMERS

    //intake stuff

    public static long intakeOutputtingTimer;
    public static long intakeAfterTransferClosedClawTimer;
    public static long intakeOutputtingTimerManual;
    public static long intakeSpinMotorMorePowerAfterTakingTimer;



    //outtake stuff
    public static long outtakeSpecimenAfterScoreTimer;
    public static long outtakeAfterBasketSampleScoreTimer;
    public static long outtakeAfterHasClosedClawAtWallSpecimenTimer;



    public static void resetStuff() {
        // MISC
        PIDincrement = 0;
        gravityAdder = 0;

        // intake
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeExtendMotorTargetPos = 0;
        intakeTargetPosAdder = 0;

        // outtake
        outtakePivotServoPos = outtakePivotServoTransferPos;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakeExtendMotorTargetPos = 0;
        outtakeTargetPosAdder = 0;

        // BOOLEANS
        isYellowSampleNotGood = false;

        // is pressed
        isPressedA1 = false;
        isPressedA2 = false;
        isPressedB1 = false;
        isPressedB2 = false;
        isPressedX1 = false;
        isPressedX2 = false;
        isPressedY1 = false;
        isPressedY2 = false;
        isPressedDL1 = false;

        // intake stuff
        isAfterIntakeBeenDownColecting = false;
        isIntakeOutputting = false;
        isAfterBotHasBeenOutputting = false;
        isAfterOuttakeClawClosedAfterTransfer = false;
        isIntakeOutputtingManual = false;

        // outtake stuff
        isAfterOuttakeScoredSpecimen = false;
        isAfterOuttakeScoredBasketSample = false;
        isAfterOuttakeClosedClawAtWallSpecimen = false;
        isAtStateOfLettingBasketSampleGo = false;

        // LONGS / TIMERS
        outtakeSpecimenAfterScoreTimer = 0;
        outtakeAfterBasketSampleScoreTimer = 0;
        outtakeAfterHasClosedClawAtWallSpecimenTimer = 0;
        intakeOutputtingTimer = 0;
        intakeAfterTransferClosedClawTimer = 0;
    }
}