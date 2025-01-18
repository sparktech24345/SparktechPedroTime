package pedroPathing.States;


@com.acmerobotics.dashboard.config.Config
public class PositionStorage {


    // always remember Pisicaaaa
    public static double intakeTargetPosAdder=0;
    public static boolean timerNotSet = false;
    public static double intakeTimeTransferAdder = 15;
    public static double outakeSampleRetracted = 5;
    public static double intakeTransferMarjeOfErrorBeforeTransfer=10; //42
    public static long bambuTransferTimer;
    public static boolean wasBambuExtended = false;
    public static double intakeActualZero = 70; //old 75
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
    public static double outakeTransferPos= 70;
    public static long noWiglyTransferTimer;
    public static boolean noWiglyPls = false;
    public static boolean doOnceyTransfer = false;
    public static double outakeArmTransferPos = 40;
    public static long intakeExtraSpinTimer;
    public static long intakeExtraSpinOUTPUTTimer;
    public static boolean intakeExtraSpinDoOnce = false;
    public static boolean intakeExtraSpinOUTPUTDoOnce = false;
    public static double OutTime = 50;
    public static boolean TransferDisabled = false;

    public static void resetStuff(){
        TransferDisabled = false;
        intakeExtraSpinDoOnce = false;
        outakeArmTransferPos = 40;
        doOnceyTransfer = false;
        noWiglyPls = false;
        outakeTransferPos= 70;
        isHeldBascket = false;
        shouldTransfer = false;
        afterSpecimenOpenTime = 80;
        stateStringOutake = "not set";
        stateStringIntake = "not set";
        goToPickUp = false;
        wasOutputHM = false;
        IntakeServoColectPos = 320; //from 300
        willTransfer = true;
        intakeTargetPosAdder=0;
        timerNotSet = false;
        intakeTimeTransferAdder = 15;
        outakeSampleRetracted = 5;
        intakeTransferMarjeOfErrorBeforeTransfer = 10 ;//old 48
        wasBambuExtended = false;
        intakeActualZero = 70; //old 75
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
    }

}