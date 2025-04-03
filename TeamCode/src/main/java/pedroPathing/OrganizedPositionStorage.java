package pedroPathing;

import static pedroPathing.newOld.Toggle.*;

@com.acmerobotics.dashboard.config.Config
public class OrganizedPositionStorage {

    // Integer variables
    public static int intakeOutputTruBotPosition = 15;
    public static int servoextended = 150;
    public static int teoBdayCase = 0;

    // Long variables
    public static long autoTimer;
    public static long bambuTransferTimer;
    public static long colortimer;
    public static long intakeExtraSpinOUTPUTTimer;
    public static long intakeExtraSpinTimer;
    public static long noWiglyTransferTimer;
    public static long outPutTimer;
    public static long startingTimer;
    public static long startingTimer2;
    public static long startingTimer5;
    public static long spinyOutputTimer;
    public static long timerSticlaDeApa;



    // Double variables
    public static double addedTimer = 0;
    public static double afterSpecimenOpenTime = 80;
    public static double gravityAdder = 0;
    public static double intakeActualZero = 60; // old 75
    public static double intakeMotorPickUpPower = 0;
    public static double intakeRotateAfterRo2Trasfer = 160;
    public static double intakeRotateServoPosition = 40; // old 35
    public static double intakeTargetPos = 0;
    public static double intakeTargetPosAdder = 20;
    public static double intakeTimeTransferAdder = 15;
    public static double intakeTransferAngles = 40; // old 35
    public static double intakeTransferMarjeOfErrorBeforeTransfer = 10; // 42 // 10 more recent
    public static double intakeTransferSlidersAdder = 0;
    public static double intakeRo2SmashPos = 45;
    public static double outakeArmServoPosition = 60;
    public static double outakeArmTransferPos = 40;
    public static double outakeRotateServoPosition = 105;
    public static double outakeRotateServoPositionDefault = 60; // 40 increments
    public static double outakeSampleRetracted = 12;
    public static double outakeSampleServoPosition = 0;
    public static double outakeTargetPos = 0;
    public static double outakeTargetPosAdder = 0;
    public static double outakeTransferPos = 80;
    public static double outtakeArmSpecimenPut = 323;
    public static double outtakeArmServoPosAtRo2v2TransferPickUp = 43;
    public static double pickUpAngleRo2V2Adder = 105;
    public static double extendABitAfterRo2TransferPos = intakeTransferSlidersAdder + 100;
    public static double frontLeftPowerCat = 0;
    public static double frontRightPowerCat = 0;
    public static double backLeftPowerCat = 0;
    public static double backRightPowerCat = 0;
    public static double OutTime = 50;



    // Boolean variables
    public static boolean GoOutSample = false;
    public static boolean autoSlowdown = false;
    public static boolean didTransfer = false;
    public static boolean extendABitAfterRo2Transfer = false;
    public static boolean hangTime = false;
    public static boolean isHeldBascket = false;
    public static boolean isIntakeStateExtended = false;
    public static boolean isIntakeStateRectracted = false;
    public static boolean isOutputting = false;
    public static boolean isRaised = false;
    public static boolean isOutputinHM = false;
    public static boolean noWiglyPls = false;
    public static boolean PickyUppyOnce = false;
    public static boolean shouldBeRaised = false;
    public static boolean shouldTransfer = false;
    public static boolean SpitOutSampleHM = false;
    public static boolean SpitOutSampleHM2 = false;
    public static boolean stopMulthiread = true;
    public static boolean takeWhileDisabled = false;
    public static boolean timerNotSet = false;
    public static boolean transferDisabled = false;
    public static boolean transferTimerInit = false;
    public static boolean wasActivePastActiveIntake = false;
    public static boolean wasActiveintake = false;
    public static boolean wasBambuExtended = false;
    public static boolean wasBadSample = false;
    public static boolean wascolor = false;
    public static boolean wasIntakeStateExtended = false;
    public static boolean wasOutputHM2 = false;
    public static boolean wasOuttakeStateBascket = false;
    public static boolean wasOuttakeStateSpecimen = false;
    public static boolean wasisOuttakeStateSpecimen = false;
    public static boolean spinyOutputToggle = false;
    public static boolean someExtraThingDoOnce = false;
    public static boolean doOnceyTransfer = false;
    public static boolean intakeExtraSpinDoOnce = false;
    public static boolean intakeExtraSpinOUTPUTDoOnce = false;
    public static boolean intakeShouldRetractAfterTransfer = false;
    public static boolean intakeShouldRetractAfterTransferTimerToggle = false;
    public static boolean wasOutputHM = false;
    public static boolean isOuttakeStateStandbyWithSample = false;
    public static boolean isOuttakeStateSamplePickUp = false;
    public static boolean isOuttakeStateBascket = false;

    // String variables
    public static String stateStringIntake = "not set";
    public static String stateStringOutake = "not set";
    public static String team = "TeamNotSet";

    public static void resetStuff() {
        // Reset Integer variables
        intakeOutputTruBotPosition = 15;
        teoBdayCase = 0;
        servoextended = 150;

        // Reset Long variables
        startingTimer = 0;
        startingTimer2 = 0;
        startingTimer5 = 0;
        colortimer = 0;
        outPutTimer = 0;
        spinyOutputTimer = 0;
        noWiglyTransferTimer = 0;
        intakeExtraSpinTimer = 0;
        intakeExtraSpinOUTPUTTimer = 0;
        autoTimer = 0;
        timerSticlaDeApa = 0;

        // Reset Double variables

        //intake
        addedTimer = 0;
        afterSpecimenOpenTime = 80;
        gravityAdder = 0;
        intakeActualZero = 60; // old 75
        intakeMotorPickUpPower = 0;
        intakeRotateAfterRo2Trasfer = 160;
        intakeRotateServoPosition = 40; // old 35
        intakeTargetPos = 0;
        intakeTargetPosAdder = 20;
        intakeTimeTransferAdder = 15;
        intakeTransferAngles = 40; // old 35
        intakeTransferMarjeOfErrorBeforeTransfer = 10; // 42 // 10 more recent
        intakeTransferSlidersAdder = 0;
        intakeRo2SmashPos = 45;
        extendABitAfterRo2TransferPos = intakeTransferSlidersAdder + 100;
        pickUpAngleRo2V2Adder = 105;

        //outtaake
        outakeArmServoPosition = 60;
        outakeArmTransferPos = 40;
        outakeRotateServoPosition = 105;
        outakeRotateServoPositionDefault = 60; // 40 increments
        outakeSampleRetracted = 12;
        outakeSampleServoPosition = 0;
        outakeTargetPos = 0;
        outakeTargetPosAdder = 0;
        outakeTransferPos = 80;
        outtakeArmSpecimenPut = 323;
        outtakeArmServoPosAtRo2v2TransferPickUp = 43;

        //motor
        frontLeftPowerCat = 0;
        frontRightPowerCat = 0;
        backLeftPowerCat = 0;
        backRightPowerCat = 0;

        //idk
        OutTime = 50;

        // Reset Boolean variables
        GoOutSample = false;
        autoSlowdown = false;
        didTransfer = false;
        extendABitAfterRo2Transfer = false;
        hangTime = false;
        isHeldBascket = false;
        isIntakeStateExtended = false;
        isIntakeStateRectracted = false;
        isOutputting = false;
        isRaised = false;
        isOutputinHM = false;
        noWiglyPls = false;
        PickyUppyOnce = false;
        shouldBeRaised = false;
        shouldTransfer = false;
        SpitOutSampleHM = false;
        SpitOutSampleHM2 = false;
        stopMulthiread = true;
        takeWhileDisabled = false;
        timerNotSet = false;
        transferDisabled = false;
        transferTimerInit = false;
        wasActivePastActiveIntake = false;
        wasActiveintake = false;
        wasBambuExtended = false;
        wasBadSample = false;
        wascolor = false;
        wasIntakeStateExtended = false;
        wasOutputHM2 = false;
        wasOuttakeStateBascket = false;
        wasOuttakeStateSpecimen = false;
        wasisOuttakeStateSpecimen = false;
        spinyOutputToggle = false;
        someExtraThingDoOnce = false;
        doOnceyTransfer = false;
        intakeExtraSpinDoOnce = false;
        intakeExtraSpinOUTPUTDoOnce = false;
        intakeShouldRetractAfterTransfer = false;
        intakeShouldRetractAfterTransferTimerToggle = false;
        wasOutputHM = false;
        wasOuttakeStateSpecimen = false;
        isOuttakeStateStandbyWithSample = false;
        isOuttakeStateSamplePickUp = false;
        isOuttakeStateBascket = false;

        // Reset String variables
        stateStringIntake = "not set";
        stateStringOutake = "not set";
        team = "TeamNotSet";
    }
}