package pedroPathing;


import static pedroPathing.Toggle.*;

@com.acmerobotics.dashboard.config.Config
public class PositionStorage {


    // always remember Pisicaaaa
    public static double intakeTargetPosAdder=20;
    public static boolean timerNotSet = false;
    public static double intakeTimeTransferAdder = 15;
    public static double outakeSampleRetracted = 12;
    public static double intakeTransferMarjeOfErrorBeforeTransfer=10; //42 //10 more recent
    public static long bambuTransferTimer;
    public static boolean wasBambuExtended = false;
    public static double intakeActualZero = 60; //old 75
    public static double intakeTransferAngles = 40; //old 35
    public static double intakeRotateServoPosition = intakeTransferAngles;
    public static double outakeArmServoPosition = 60;
    public static double outakeSampleServoPosition = 0;
    public static double intakeTargetPos = 0;
    public static double outakeTargetPos =0;
    public static double outakeTargetPosAdder =0;
    public static double outakeRotateServoPosition = 105;
    public static double outakeRotateServoPositionDefault = 60; // 40 increments
    public static double intakeMotorPickUpPower = 0;
    public static boolean isIntakeStateExtended = false;
    public static boolean isIntakeStateRectracted = false;
    public static boolean isOuttakeStateStandbyWithSample = false;
    public static boolean isOuttakeStateSamplePickUp = false;
    public static boolean isOuttakeStateBascket = false;
    public static boolean isOuttakeStateSpecimen = false;
    public static boolean wasIntakeStateExtended = false;
    public static boolean wasActiveintake =false;
    public static boolean wasActivePastActiveIntake = false;
    public static boolean wasOuttakeStateBascket = false;
    public static boolean wasOuttakeStateSpecimen = false;
    public static boolean wasisOuttakeStateSpecimen = false;
    public static boolean spinyOutputToggle = false;
    public static int servoextended = 150;
    public static double  armServoPos =0;
    public static long startingTimer;
    public static boolean isPressedA1 = false;
    public static boolean isPressedA2 = false;
    public static boolean isPressedB1 = false;
    public static boolean isPressedB2 = false;
    public static boolean isPressedX1 = false;
    public static boolean isPressedX2 = false;
    public static boolean isPressedY1 = false;
    public static boolean isPressedY2 = false;
    public static boolean isPressedDL1 = false;
    public static boolean telemetryOhNo= false;
    public static double frontRightPowerCat=0;
    public static double backRightPowerCat=0;
    public static double frontLeftPowerCat=0;
    public static double backLeftPowerCat=0;
    public static double gravityAdder = 0;
    public static String team = "TeamNotSet";
    public static boolean autoSlowdown = false;
    public static boolean willTransfer = true;
    public static double IntakeServoColectPos = 300;

    public static boolean GoOutSample = false;
    public static boolean wascolor = false;
    public static double PIDincrement =1;
    public static boolean wasBadSample = false;
    public static double intakeExtendedTargetPos = 0;
    public static long startingTimer2;
    public static long startingTimer5;
    public static long colortimer;
    public static long outPutTimer;
    public static  long spinyOutputTimer;
    public static boolean wasOutputHM =false;
    public static boolean goToPickUp = false;
    public static String stateStringOutake = "not set";
    public static String stateStringIntake = "not set";
    public static double afterSpecimenOpenTime = 80;
    public static boolean shouldTransfer = false;
    public static double rememberPosOfServoOut = 0;
    public static boolean isHeldBascket = false;
    public static double outakeTransferPos= 80;
    public static long noWiglyTransferTimer;
    public static boolean noWiglyPls = false;
    public static boolean doOnceyTransfer = false;
    public static double outakeArmTransferPos = 40;
    public static long intakeExtraSpinTimer;
    public static long intakeExtraSpinOUTPUTTimer;
    public static boolean intakeExtraSpinDoOnce = false;
    public static boolean intakeExtraSpinOUTPUTDoOnce = false;
    public static double OutTime = 50;
    public static boolean transferDisabled = false;
    public static long SpitOutSampleHMTimer;
    public static boolean SpitOutSampleHM = false;
    public static boolean SpitOutSampleHM2 = false;
    public static boolean PickyUppyOnce = false;
    public static double intakeSlidersRo2Transfer = 205;
    public static double intakeRotateAfterRo2Trasfer = 160;
    public static boolean extendABitAfterRo2Transfer = false;
    public static double extendABitAfterRo2TransferPos = intakeSlidersRo2Transfer + 100;
    public static boolean intakeShouldRetractAfterTransfer = false;
    public static boolean intakeShouldRetractAfterTransferTimerToggle = false;
    public static long intakeShouldRetractAfterTransferTimer;
    public static double pickUpAngleRo2V2Adder = 105;
    public static double intakeRo2SmashPos = 45;
    public static boolean someExtraThingDoOnce = false;
    public static boolean transferTimerInit = false;
    public static double TransferTimer;
    public static boolean HeadUpIntake = false;
    public static boolean stopMulthiread = true;
    public static double outtakeArmServoPosAtRo2v2TransferPickUp = 43;
    public static double outtakeArmSpecimenPut =323;
    public static double OuttakeArmWallPickUpPosition = 64;
    public static boolean DontDoTransferBeforeTransfer = false;
    public static boolean hangTime = false;
    public static long autoTimer;
    public static double IntakeWallPickUpPosition = 0;
    public static double intakeRotateForWallPickUp = 55;
    public static boolean takeWhileDisabled = false;
    public static boolean isOutputting = false;
    public static boolean wasOutputHM2 = false;
    public static long timerSticlaDeApa;
    public static boolean didTransfer = false;
    public static boolean isRaised = false;
    public static boolean shouldBeRaised = false;
    public static boolean isOutputinHM = false;
    public static double addedTimer=0;
    public static double intakeTransferSlidersAdder=0;
    public static boolean someOtherBollean = false;

    public static void resetStuff(){
        someOtherBollean = false;
        intakeTransferSlidersAdder=0;
        addedTimer =0;
        isOutputinHM = false;
        shouldBeRaised = false;
        isRaised = false;
        didTransfer = false;
        wasOutputHM2 = false;
        isOutputting = false;
        takeWhileDisabled = false;
        intakeRotateForWallPickUp = 55;
        isPressedDL1 = false;
        hangTime = false;
        DontDoTransferBeforeTransfer = false;
        OuttakeArmWallPickUpPosition = 64;
        outtakeArmSpecimenPut = 323;
        outtakeArmServoPosAtRo2v2TransferPickUp = 43;
        //stopMulthiread = true;
        HeadUpIntake = false;
        transferTimerInit = false;
        someExtraThingDoOnce = false;
        intakeRo2SmashPos = 45;
        pickUpAngleRo2V2Adder = 105;
        intakeShouldRetractAfterTransferTimerToggle = false;
        intakeShouldRetractAfterTransfer = false;
        intakeRotateAfterRo2Trasfer = 160;
        extendABitAfterRo2TransferPos = intakeSlidersRo2Transfer + 100;
        extendABitAfterRo2Transfer= false;
        intakeSlidersRo2Transfer = 205;
        SpitOutSampleHM = false;
        SpitOutSampleHM2 = false;
        PickyUppyOnce = false;
        transferDisabled = false;
        intakeExtraSpinDoOnce = false;
        outakeArmTransferPos = 40;
        doOnceyTransfer = false;
        noWiglyPls = false;
        outakeTransferPos= 80;
        isHeldBascket = false;
        shouldTransfer = false;
        afterSpecimenOpenTime = 80;
        stateStringOutake = "not set";
        stateStringIntake = "not set";
        goToPickUp = false;
        wasOutputHM = false;
        IntakeServoColectPos = 235; //from 320 from 300
        willTransfer = true;
        intakeTargetPosAdder=20;
        timerNotSet = false;
        intakeTimeTransferAdder = 15;
        outakeSampleRetracted = 12;
        intakeTransferMarjeOfErrorBeforeTransfer = 10 ;//old 48 //10 more recent
        wasBambuExtended = false;
        intakeActualZero = 60; //old 75
        intakeTransferAngles = 40; //old 35
        intakeRotateServoPosition = intakeTransferAngles;
        outakeArmServoPosition = 60;
        outakeSampleServoPosition = 0;
        intakeTargetPos = 0;
        outakeTargetPos = 0;
        outakeTargetPosAdder = 0;
        outakeRotateServoPosition = 105;
        outakeRotateServoPositionDefault = 60; // 40 increments
        intakeMotorPickUpPower = 0;
        isIntakeStateExtended = false;
        isIntakeStateRectracted = false;
        isOuttakeStateStandbyWithSample = false;
        isOuttakeStateSamplePickUp = false;
        isOuttakeStateBascket = false;
        isOuttakeStateSpecimen = false;
        wasIntakeStateExtended = false;
        wasActiveintake = false;
        wasActivePastActiveIntake = false;
        wasOuttakeStateBascket = false;
        wasOuttakeStateSpecimen = false;
        wasisOuttakeStateSpecimen = false;
        spinyOutputToggle = false;
        servoextended = 150;
        armServoPos = 0;
        isPressedA1 = false;
        isPressedA2 = false;
        isPressedB1 = false;
        isPressedB2 = false;
        isPressedX1 = false;
        isPressedX2 = false;
        isPressedY1 = false;
        isPressedY2 = false;
        telemetryOhNo = false;
        frontRightPowerCat = 0;
        backRightPowerCat = 0;
        frontLeftPowerCat = 0;
        backLeftPowerCat = 0;
        gravityAdder = 0;
        team = "TeamNotSet";
        autoSlowdown = false;

        toggle_var = false;
        toggled = false;
    }

}