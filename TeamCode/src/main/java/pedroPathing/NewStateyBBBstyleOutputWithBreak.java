package pedroPathing;


import static pedroPathing.ClassWithStates.ColorCompare;
import static pedroPathing.ClassWithStates.colorList;
import static pedroPathing.ClassWithStates.colorSensorOutty;
import static pedroPathing.ClassWithStates.currentStateOfSampleInIntake;
import static pedroPathing.ClassWithStates.currentTeam;
import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.ClassWithStates.intakeCabinDownCollecting;
import static pedroPathing.ClassWithStates.intakeCabinDownOutputting;
import static pedroPathing.ClassWithStates.intakeCabinFullInBot;
import static pedroPathing.ClassWithStates.intakeCabinFullInBotOutputting;
import static pedroPathing.ClassWithStates.intakeCabinState;
import static pedroPathing.ClassWithStates.intakeCabinStates;
import static pedroPathing.ClassWithStates.intakeCabinTransferPosition;
import static pedroPathing.ClassWithStates.intakeCabinTransferPositionWithPower;
import static pedroPathing.ClassWithStates.intakeExtended1out4;
import static pedroPathing.ClassWithStates.intakeExtended2out4;
import static pedroPathing.ClassWithStates.intakeExtended3out4;
import static pedroPathing.ClassWithStates.intakeExtended4out4;
import static pedroPathing.ClassWithStates.intakeRetracted;
import static pedroPathing.ClassWithStates.intakeState;
import static pedroPathing.ClassWithStates.intakeStates;
import static pedroPathing.ClassWithStates.outtakeBasket;
import static pedroPathing.ClassWithStates.outtakeSpecimenHang;
import static pedroPathing.ClassWithStates.outtakeStandByWithoutExtensions;
import static pedroPathing.ClassWithStates.outtakeState;
import static pedroPathing.ClassWithStates.outtakeStates;
import static pedroPathing.ClassWithStates.outtakeTransfer;
import static pedroPathing.ClassWithStates.outtakeWallPickUpNew;
import static pedroPathing.OrganizedPositionStorage.PIDincrement;
import static pedroPathing.OrganizedPositionStorage.basketStandbyState;
import static pedroPathing.OrganizedPositionStorage.beforeOuttakeGoDownTimer;
import static pedroPathing.OrganizedPositionStorage.chassisBackLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisBackRightPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontRightPow;
import static pedroPathing.OrganizedPositionStorage.hasIntakeOutputedTruBot;
import static pedroPathing.OrganizedPositionStorage.hasPressedXTimer;
import static pedroPathing.OrganizedPositionStorage.hasSmolOutputed;
import static pedroPathing.OrganizedPositionStorage.hasSmolOutputedTimer;
import static pedroPathing.OrganizedPositionStorage.intakeAfterTransferClosedClawTimer;
import static pedroPathing.OrganizedPositionStorage.intakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakeOutputtingTimer;
import static pedroPathing.OrganizedPositionStorage.intakeOutputtingTimerManual;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.intakeSpinMotorMorePowerAfterTakingTimer;
import static pedroPathing.OrganizedPositionStorage.intakeSpinMotorPow;
import static pedroPathing.OrganizedPositionStorage.intakeTargetPosAdder;
import static pedroPathing.OrganizedPositionStorage.isAfterIntakeBeenDownColecting;
import static pedroPathing.OrganizedPositionStorage.isAfterOuttakeClawClosedAfterTransfer;
import static pedroPathing.OrganizedPositionStorage.isAfterOuttakeClosedClawAtWallSpecimen;
import static pedroPathing.OrganizedPositionStorage.isAfterOuttakeScoredBasketSample;
import static pedroPathing.OrganizedPositionStorage.isAfterOuttakeScoredSpecimen;
import static pedroPathing.OrganizedPositionStorage.isAfterTakingTakeySpiny;
import static pedroPathing.OrganizedPositionStorage.isAtStateOfLettingBasketSampleGo;
import static pedroPathing.OrganizedPositionStorage.isColorSensorNotInUse;
import static pedroPathing.OrganizedPositionStorage.isInLowerBasketState;
import static pedroPathing.OrganizedPositionStorage.isInNeedToGoToSpecimenTransferPos;
import static pedroPathing.OrganizedPositionStorage.isInPositionToRaiseOuttakeInOrderToEvadeIntake;
import static pedroPathing.OrganizedPositionStorage.isInSpecimenState;
import static pedroPathing.OrganizedPositionStorage.isIntakeOutputting;
import static pedroPathing.OrganizedPositionStorage.isIntakeOutputtingManual;
import static pedroPathing.OrganizedPositionStorage.isIntakeSpinMOtorAfterJustTaking;
import static pedroPathing.OrganizedPositionStorage.isOuttakeAfterOutputedTruBot;
import static pedroPathing.OrganizedPositionStorage.isOuttakeAfterOutputedTruBotTimer;
import static pedroPathing.OrganizedPositionStorage.isOuttakeInPositionToGoDown;
import static pedroPathing.OrganizedPositionStorage.isPressedA1;
import static pedroPathing.OrganizedPositionStorage.isPressedA2;
import static pedroPathing.OrganizedPositionStorage.isPressedB1;
import static pedroPathing.OrganizedPositionStorage.isPressedB2;
import static pedroPathing.OrganizedPositionStorage.isPressedD2Up;
import static pedroPathing.OrganizedPositionStorage.isPressedX1;
import static pedroPathing.OrganizedPositionStorage.isPressedX2;
import static pedroPathing.OrganizedPositionStorage.isPressedY2;
import static pedroPathing.OrganizedPositionStorage.isTimeToRefreshOutptingTime;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
import static pedroPathing.OrganizedPositionStorage.justTransfered;
import static pedroPathing.OrganizedPositionStorage.needsToExtraExtend;
import static pedroPathing.OrganizedPositionStorage.outtakeAfterBasketSampleScoreTimer;
import static pedroPathing.OrganizedPositionStorage.outtakeAfterHasClosedClawAtWallSpecimenTimer;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoExtendedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoExtraExtendedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoRetractedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.outtakeIsInNeedToExtraExtendClawTimer;
import static pedroPathing.OrganizedPositionStorage.outtakeMotorActualZeroPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoTransferPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoWallPickupPos;
import static pedroPathing.OrganizedPositionStorage.outtakeSlidersWallPickPos;
import static pedroPathing.OrganizedPositionStorage.outtakeSpecimenAfterScoreTimer;
import static pedroPathing.OrganizedPositionStorage.outtakeTargetPosAdder;
import static pedroPathing.OrganizedPositionStorage.shouldAutoCollect;
import static pedroPathing.OrganizedPositionStorage.shouldStopIntakeCabinSpinningAfterTakig;
import static pedroPathing.OrganizedPositionStorage.shouldStopIntakeCabinSpinningAfterTakigTimer;
import static pedroPathing.OrganizedPositionStorage.timeSinceStartedMovingForTruBotOutput;
import static pedroPathing.OrganizedPositionStorage.waitingForOuttakeToEvadeIntakeTimer;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import pedroPathing.PIDStorageAndUse.NewPidsController;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "BBBNewStatesOutputWithBreak", group = "Linear OpMode")
public class NewStateyBBBstyleOutputWithBreak extends LinearOpMode {

    /// BRAKE
    ///

    /// BRAKE CONFIGS
    ///
    public static double SPEED_CALC_TIME = 100;
    public static double Y_BRAKE_COEFF_P = 0.00013;
    public static double X_BRAKE_COEFF_P = 0.00013;
    public static double Y_BRAKE_COEFF_D = 0.00013 / 8;
    public static double X_BRAKE_COEFF_D = 0.00013 / 8;
    public static double X_HOLD_COEFF_P = 0.00100;
    public static double Y_HOLD_COEFF_P = 0.00100;

    /// CALC VALUES

    double ySpeed = 0;
    double xSpeed = 0;

    double lastYSpeed = 0;
    double lastXSpeed = 0;

    double yAccel = 0;
    double xAccel = 0;

    double turnSpeed = 0;
    long timeUntilBreak;

    /// HELPERS
    ///

    double lastTickTime = 0;
    double lastFrontLeftPos = 0;
    double lastFrontRightPos = 0;
    double lastBackLeftPos = 0;
    double lastBackRightPos = 0;

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    MultipleTelemetry tel;
    long current_time = System.nanoTime();
    double slowyDownyManal = 0.4;
    double slowyDownyAuto = 0.5;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    private volatile boolean telemetryThreadRunning = true;



    public void calculateSpeeds(){
        double tickTime = System.currentTimeMillis();

        double frontLeftPos = -frontLeftMotor.getCurrentPosition();
        double frontRightPos = frontRightMotor.getCurrentPosition();
        double backLeftPos = -backLeftMotor.getCurrentPosition();
        double backRightPos = backRightMotor.getCurrentPosition();

        double dFL = frontLeftPos - lastFrontLeftPos;
        double dFR = frontRightPos - lastFrontRightPos;
        double dBL = backLeftPos - lastBackLeftPos;
        double dBR = backRightPos - lastBackRightPos;

        double dt = (tickTime - lastTickTime) / 1000.0; // Convert ms to seconds

        ySpeed = (dFL + dFR + dBL + dBR) / 4.0 / dt;
        xSpeed = (-dFL + dFR + dBL - dBR) / 4.0 / dt;

        yAccel = (ySpeed - lastYSpeed) / dt;
        xAccel = (xSpeed - lastXSpeed) / dt;

        if (ySpeed < 65)
            ySpeed = 0;
        if(xSpeed < 65)
            xSpeed = 0;

        // Optional: rotation speed
        turnSpeed = (-dFL + dFR - dBL + dBR) / 4.0 / dt;

        lastTickTime = tickTime;

        lastFrontLeftPos = frontLeftPos;
        lastFrontRightPos = frontRightPos;
        lastBackLeftPos = backLeftPos;
        lastBackRightPos = backRightPos;

        lastXSpeed = xSpeed;
        lastYSpeed = ySpeed;

    }


    @Override
    public void runOpMode() throws InterruptedException {

        OrganizedPositionStorage.resetStuff();

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        backLeftMotor = hardwareMap.dcMotor.get("backleft");
        frontRightMotor = hardwareMap.dcMotor.get("frontright");
        backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");


        //declare servos
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());



        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);


        Thread telemetryThread = new Thread(() -> {
            while (telemetryThreadRunning && !isStopRequested()) {
                tel.addData("isAlive", true);
                tel.update();


                //Set servo Positions
                intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
                outakeArmServo.setPosition(outtakePivotServoPos / 328);
                outakeSampleServo.setPosition(outtakeClawServoPos / 360);

                try {
                    Thread.sleep(200); // update every 200ms
                } catch (InterruptedException e) {
                    // optional: break on interrupt
                    break;
                }
            }
        });

        telemetryThread.start();



        tel.addData("current team color",currentTeam);
        tel.update();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()) {
            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            ///gamepad2
            double intakeinput = gamepad2.left_stick_y;


            if(gamepad2.a) isPressedA2 = true;
            if(!gamepad2.a && isPressedA2){
                isPressedA2 = false;
                timeUntilBreak = System.currentTimeMillis();
            }
            if(timeUntilBreak + 600 > System.currentTimeMillis())
                vertical += 1;





            //Selectare Echipa
            if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
                currentTeam = colorList.blue;
            if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
                currentTeam = colorList.red;

            if(Toggle.FirsToggle(gamepad1.left_trigger >= 0.4 && gamepad1.right_trigger >=0.4)){
                horizontal = - horizontal;
                vertical = - vertical;
            }

            colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);


            outtakeTargetPosAdder -= 1.5*gamepad2.left_stick_y;




            //interesting slowdown

            if(intakeState == intakeStates.intakeExtended2out4 ||
                    intakeState == intakeStates.intakeExtended3out4 ||
                    intakeState == intakeStates.intakeExtended4out4
            ){
                pivot = pivot * slowyDownyAuto;
            }


            ///CONTROLS

            //PICK UP
            /*
            double timeAtTransfer = 0;
            boolean isYetToGrab = false;
            */
            if(gamepad1.a){
                isPressedA1 = true;
            }
            if(!gamepad1.a && isPressedA1){
                if(intakeCabinState == intakeCabinStates.intakeCabinFullInBot && (intakeState==intakeStates.intakeRetracted || intakeState == intakeStates.intakeExtended1out4)){
                    isInPositionToRaiseOuttakeInOrderToEvadeIntake = true;
                    waitingForOuttakeToEvadeIntakeTimer = System.currentTimeMillis();
                    outtakeSpecimenHang();
                }
                else if(!(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting) && !(intakeCabinState == intakeCabinStates.intakeCabinDownOutputting)) {
                    intakeCabinDownCollecting();
                    if(!isInSpecimenState)
                        outtakeTransfer();
                    else outtakeWallPickUpNew();
                    isAfterIntakeBeenDownColecting = true;
                }
                else {
                    /*
                    intakeRetracted();
                    intakeCabinTransferPositionWithPower();
                    if(!isInSpecimenState) {
                        isIntakeSpinMOtorAfterJustTaking = true;
                        intakeSpinMotorMorePowerAfterTakingTimer = System.currentTimeMillis();
                        outtakeTransfer();
                    }
                    isAfterIntakeBeenDownColecting = false;
                    //*/
                    shouldAutoCollect = true;
                }

                isPressedA1 = false;
            }


            if(gamepad2.x) isPressedX2 = true;
            if(!gamepad2.x && isPressedX2){
                outtakeStandByWithoutExtensions();
                isInPositionToRaiseOuttakeInOrderToEvadeIntake = false;
                intakeRetracted();
                intakeCabinTransferPosition();
                isPressedX2 = false;
            }


            //SPECIMEN
            if(gamepad1.b) isPressedB1 = true;
            if(!gamepad1.b && isPressedB1){
                if(!(outtakeState == outtakeStates.outtakeSpecimenHang)) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    isAfterOuttakeClosedClawAtWallSpecimen = true;
                    outtakeAfterHasClosedClawAtWallSpecimenTimer = System.currentTimeMillis();
                }
                else{
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    outtakeSpecimenAfterScoreTimer = System.currentTimeMillis();
                    isAfterOuttakeScoredSpecimen = true;
                }
                isPressedB1 = false;
            }
            if(isAfterOuttakeClosedClawAtWallSpecimen && outtakeAfterHasClosedClawAtWallSpecimenTimer + 300 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeSpecimenHang();
                isAfterOuttakeClosedClawAtWallSpecimen = false;
            }
            if(isAfterOuttakeScoredSpecimen && outtakeSpecimenAfterScoreTimer + 300 < System.currentTimeMillis()){
                outtakeWallPickUpNew();
                isAfterOuttakeScoredSpecimen = false;
                outtakeIsInNeedToExtraExtendClawTimer = System.currentTimeMillis();
            }
            if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
                needsToExtraExtend = false;
                outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
            }


            //BASKET SCORING
            if(gamepad1.x) {
                isPressedX1 = true;
                hasPressedXTimer = System.currentTimeMillis();
            }
            if(!gamepad1.x && isPressedX1){
                if(!(outtakeState == outtakeStates.outtakeBasket)){

                    isAfterOuttakeClawClosedAfterTransfer = true;
                    intakeAfterTransferClosedClawTimer = System.currentTimeMillis();
                    isAtStateOfLettingBasketSampleGo = true;
                }
                else intakeRetracted();
                isPressedX1 = false;
            }
            if(isAfterOuttakeClawClosedAfterTransfer && intakeAfterTransferClosedClawTimer + 300 < System.currentTimeMillis() ){
                intakeRetracted();
                intakeCabinTransferPosition();
                outtakeBasket();
                isAfterOuttakeClawClosedAfterTransfer = false;
            }
            //going down after, quite complicated cuz holding to let sample go
            if(isAtStateOfLettingBasketSampleGo && gamepad1.x && outtakeState == outtakeStates.outtakeBasket){
                outtakeClawServoPos = outtakeClawServoExtendedPos;
                isAfterOuttakeScoredBasketSample = true;
                isAtStateOfLettingBasketSampleGo = false;
                outtakeAfterBasketSampleScoreTimer = System.currentTimeMillis();
            }
            if(!gamepad1.x && isAfterOuttakeScoredBasketSample) {
                outtakePivotServoPos = outtakePivotServoTransferPos;
                if(outtakeAfterBasketSampleScoreTimer + 300 < System.currentTimeMillis()) {
                    outtakeTransfer();
                    isAfterOuttakeScoredBasketSample = false;
                }
            }
            //some weird bug that can be easy fix
            if(outtakeExtendMotorTargetPos == 0 && outtakeState == outtakeStates.outtakeBasket){
                outtakeBasket();
            }


            //LOWERBASKET
            if(gamepad2.b) isPressedB2  = true;
            if(!gamepad2.b && isPressedB2){
                isInLowerBasketState = !isInLowerBasketState;
                isPressedB2 = false;
            }
            if(justTransfered && isInLowerBasketState && outtakeState == outtakeStates.outtakeSpecimenHang){
                justTransfered = false;
                //outtakeBasket();
                isPressedX1 = true;
            }




            /*
            //WALL PICK UP
            if(gamepad1.y) isPressedY1 = true;
            if(!gamepad1.y && isPressedY1){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeWallPickUpNew();
                isPressedY1 = false;
            }*/

            ///SOME STUFF

            //auto retract
            if((currentStateOfSampleInIntake == colorSensorOutty.correctSample && isAfterIntakeBeenDownColecting && !isColorSensorNotInUse) ||
                shouldAutoCollect
            ){
                shouldAutoCollect = false;
                intakeRetracted();
                //makins sure sample enetered the intake fully with a small timer
                isAfterIntakeBeenDownColecting = false;
                isIntakeSpinMOtorAfterJustTaking = true;
                basketStandbyState = 0;
                intakeSpinMotorMorePowerAfterTakingTimer = System.currentTimeMillis();


                outtakeClawServoPos = outtakeClawServoExtendedPos;
            }


            if(basketStandbyState == 0 && isIntakeSpinMOtorAfterJustTaking && intakeSpinMotorMorePowerAfterTakingTimer + 100 < System.currentTimeMillis()){
                isIntakeSpinMOtorAfterJustTaking = false;
                if(isAfterTakingTakeySpiny) {
                    intakeCabinTransferPositionWithPower();
                    isAfterTakingTakeySpiny = false;
                }



                if(!isInSpecimenState && outakeLeftMotor.getCurrentPosition()> -20) {
                    outtakeTransfer();
                    basketStandbyState++;
                }
                outtakeClawServoPos = outtakeClawServoExtendedPos;

            }
            if(basketStandbyState == 1 && intakeSpinMotorMorePowerAfterTakingTimer + 500 < System.currentTimeMillis()) {
                outtakeClawServoPos = outtakeClawServoRetractedPos;
                basketStandbyState++;
            }
            if(basketStandbyState == 2 && intakeSpinMotorMorePowerAfterTakingTimer + 1000 < System.currentTimeMillis()) {
                //outtakeClawServoPos = outtakeClawServoRetractedPos;
                basketStandbyState++;
                outtakeSpecimenHang();
                justTransfered = true;

                basketStandbyState = 0;
                //outtakeExtendMotorTargetPos = outtakeMotorStandByPos;

            }

            //smol give out after the extra take in


            if(shouldStopIntakeCabinSpinningAfterTakig && shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis()){
                intakeSpinMotorPow = 0.8;
                shouldStopIntakeCabinSpinningAfterTakig = false;
                hasSmolOutputed = true;
                hasSmolOutputedTimer = System.currentTimeMillis();
            }
            //and then stop the power stuff
            if(hasSmolOutputed && hasSmolOutputedTimer + 35 <System.currentTimeMillis()){
                intakeCabinTransferPosition();
                if(isInSpecimenState){
                    intakeCabinFullInBot();
                }
                hasSmolOutputed = false;
            }//*/



            //auto eject
            if(currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                intakeCabinDownOutputting();
                isIntakeOutputting = true;
                intakeOutputtingTimer = System.currentTimeMillis();
            }
            if(isIntakeOutputting && intakeOutputtingTimer + 300 < System.currentTimeMillis()){
                intakeCabinDownCollecting();
                isIntakeOutputting = false;
            }



            //manual eject
            if(gamepad1.left_bumper){
                intakeSpinMotorPow = 0.75;
                isIntakeOutputtingManual = true;
                intakeOutputtingTimerManual = System.currentTimeMillis();
            }
            if(isIntakeOutputtingManual && intakeOutputtingTimerManual + 300 < System.currentTimeMillis()){
                if(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting) intakeSpinMotorPow = 1;
                else if(intakeCabinState == intakeCabinStates.intakeCabinDownOutputting) intakeSpinMotorPow = -0.75;
                else intakeSpinMotorPow = 0;
                isIntakeOutputtingManual = false;
            }


            //chosing intake positions
            //Intake positions
            if (gamepad1.dpad_left)  intakeExtended4out4();
            if (gamepad1.dpad_down)  intakeExtended3out4();
            if (gamepad1.dpad_right) intakeExtended2out4();
            if(gamepad1.dpad_up)     intakeExtended1out4();
            if(gamepad2.left_bumper) intakeRetracted();


            //getting to specimen pick up pos without messing cables
            if(isInNeedToGoToSpecimenTransferPos && outakeLeftMotor.getCurrentPosition()<-500){
                outtakePivotServoPos = outtakePivotServoWallPickupPos;
                isInNeedToGoToSpecimenTransferPos = false;
            }

            //getting out of specimen pick up pos without messing cables
            if(isOuttakeInPositionToGoDown){
                if(outtakeState == outtakeStates.outtakeBasket){
                    isOuttakeInPositionToGoDown = false;
                    outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
                }
                else if(beforeOuttakeGoDownTimer + 400 < System.currentTimeMillis()) {
                    outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
                    isOuttakeInPositionToGoDown = false;
                }
            }

            if(isInPositionToRaiseOuttakeInOrderToEvadeIntake && waitingForOuttakeToEvadeIntakeTimer + 150 < System.currentTimeMillis()){
                intakeCabinDownCollecting();
                if(!isInSpecimenState)
                    outtakeTransfer();
                isAfterIntakeBeenDownColecting = true;
                isInPositionToRaiseOuttakeInOrderToEvadeIntake = false;
            }





            if(!(gamepad1.y)){
                isTimeToRefreshOutptingTime = true;
                if(intakeCabinState == intakeCabinStates.intakeCabinFullInBotOutputting){
                    intakeCabinFullInBot();
                    outtakeSpecimenHang();
                    outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
                }

            } else {
                if (isTimeToRefreshOutptingTime) {
                    isTimeToRefreshOutptingTime = false;
                    timeSinceStartedMovingForTruBotOutput = (long) (intakePivotServoPos - 15) * 4 + System.currentTimeMillis();
                    intakeCabinFullInBot();
                    outtakeSpecimenHang();
                    outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
                } else if (System.currentTimeMillis() > timeSinceStartedMovingForTruBotOutput) {
                    intakeCabinFullInBotOutputting();
                    outtakeSpecimenHang();
                    outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
                    hasIntakeOutputedTruBot = true;
                }
            }
            if(hasIntakeOutputedTruBot && intakeCabinState==intakeCabinStates.intakeCabinFullInBot && currentStateOfSampleInIntake == colorSensorOutty.noSample){
                hasIntakeOutputedTruBot = false;
                isOuttakeAfterOutputedTruBot = true;
                isOuttakeAfterOutputedTruBotTimer = System.currentTimeMillis();
            }
            if(isOuttakeAfterOutputedTruBot && isOuttakeAfterOutputedTruBotTimer + 300 < System.currentTimeMillis()){
                isOuttakeAfterOutputedTruBot = false;
                outtakeWallPickUpNew();
            }




            //*/

            /*if(gamepad2.b){
                if(isTimeToRefreshOutptingTime){
                    timeSinceStartedMovingForTruBotOutput = System.currentTimeMillis();
                    isTimeToRefreshOutptingTime = false;
                }
                if(!(intakeCabinState == intakeCabinStates.intakeCabinFullInBot)) {
                    intakeCabinFullInBot();
                    outtakeSpecimenHang();
                } else if (timeSinceStartedMovingForTruBotOutput + 1000 < System.currentTimeMillis() ){
                        intakeSpinMotorPow = 1;
                    }
            }
            else if (intakeCabinState == intakeCabinStates.intakeCabinFullInBot){
                    isTimeToRefreshOutptingTime = true;
                    intakeCabinFullInBot();
                    outtakeTransfer();
                    intakeSpinMotorPow = 0;
                }//*/

            if(gamepad2.a) isPressedA2 = true;
            if(!gamepad2.a && isPressedA2){
                isInSpecimenState = !isInSpecimenState;
                isYellowSampleNotGood = !isYellowSampleNotGood;
                isPressedA2 = false;
            }

            if(gamepad2.dpad_up) isPressedD2Up = true;
            if(isPressedD2Up && !gamepad2.dpad_up){
                isColorSensorNotInUse = !isColorSensorNotInUse;
                isPressedD2Up = false;
            }

            //PIDs
            PIDincrement=1;
            double intakeExtendMotorPow;
            intakeExtendMotorPow = intakeControlMotor.PIDControl (intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            //if(currentStateOfSampleInIntake == colorSensorOutty.correctSample) intakeExtendMotorPow *= 1.3;
            double outtakeExtendMotorPow;
            outtakeExtendMotorPow = NewPidsController.pidControllerOuttake(-outtakeExtendMotorTargetPos-outtakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outtakeExtendMotorPow *= PIDincrement;




            chassisFrontRightPow = (pivot - vertical - horizontal);
            chassisBackRightPow = (pivot - vertical + horizontal);
            chassisFrontLeftPow = (pivot + vertical - horizontal);
            chassisBackLeftPow = (pivot + vertical + horizontal);


            //slowdown

            //manual slowdown
            if(gamepad1.right_bumper){
                chassisFrontLeftPow *= slowyDownyManal;
                chassisBackRightPow *= slowyDownyManal;
                chassisFrontRightPow *= slowyDownyManal;
                chassisBackLeftPow *= slowyDownyManal;
            }
            //auto slowdown

            else if(// intakeState == intakeStates.intakeExtended1out4
                 outtakeState == outtakeStates.outtakeBasket
            //intakeState == intakeStates.intakeExtended2out4 ||
            //intakeState == intakeStates.intakeExtended3out4 ||
            //intakeState == intakeStates.intakeExtended4out4
            )

            //intakeCabinState == intakeCabinStates.intakeCabinDownCollecting
            {
                chassisFrontLeftPow *= slowyDownyAuto;
                chassisBackRightPow *= slowyDownyAuto;
                chassisFrontRightPow *= slowyDownyAuto;
                chassisBackLeftPow *= slowyDownyAuto;
            }//-------> FACUT DE LUCA VOICILA :)  check by Atloe


            // Toggle Claw on Y2

            if(gamepad2.y){
                isPressedY2 = true;
            }
            if(!gamepad2.y && isPressedY2){
                if(outtakeClawServoPos == outtakeClawServoRetractedPos){
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                } else {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                }
                isPressedY2 = false;
            }


            boolean brakeMode = horizontal*horizontal + vertical*vertical <= 0.225;

            if(!brakeMode){
                chassisFrontRightPow = (pivot - vertical - horizontal);
                chassisBackRightPow = (pivot - vertical + horizontal);
                chassisFrontLeftPow = (pivot + vertical - horizontal);
                chassisBackLeftPow = (pivot + vertical + horizontal);
            } else {
                chassisFrontRightPow = (pivot - ySpeed* Y_BRAKE_COEFF_P - xSpeed* X_BRAKE_COEFF_P
                                                - yAccel * Y_BRAKE_COEFF_D - xAccel * X_BRAKE_COEFF_D);
                chassisBackRightPow = (pivot - ySpeed* Y_BRAKE_COEFF_P + xSpeed* X_BRAKE_COEFF_P
                                                - yAccel * Y_BRAKE_COEFF_D + xAccel * X_BRAKE_COEFF_D);
                chassisFrontLeftPow = (pivot + ySpeed* Y_BRAKE_COEFF_P - xSpeed* X_BRAKE_COEFF_P
                                                + yAccel * Y_BRAKE_COEFF_D - xAccel * X_BRAKE_COEFF_D);
                chassisBackLeftPow = (pivot + ySpeed* Y_BRAKE_COEFF_P + xSpeed* X_BRAKE_COEFF_P
                                                + yAccel * Y_BRAKE_COEFF_D + xAccel * X_BRAKE_COEFF_D);

            }

            boolean holdMode = horizontal*horizontal + vertical*vertical <= 0.001 &&
                    Math.abs(xSpeed) < 550 && Math.abs(ySpeed) < 550;

            if(holdMode){
                chassisFrontRightPow = (pivot - ySpeed* Y_HOLD_COEFF_P - xSpeed* X_HOLD_COEFF_P);
                chassisBackRightPow = (pivot - ySpeed* Y_HOLD_COEFF_P + xSpeed* X_HOLD_COEFF_P);
                chassisFrontLeftPow = (pivot + ySpeed* Y_HOLD_COEFF_P - xSpeed* X_HOLD_COEFF_P);
                chassisBackLeftPow = (pivot + ySpeed* Y_HOLD_COEFF_P + xSpeed* X_HOLD_COEFF_P);
            }

            if (System.currentTimeMillis() >= lastTickTime + SPEED_CALC_TIME)
                calculateSpeeds();

            // set motor power
            frontLeftMotor.setPower(chassisFrontLeftPow);
            backLeftMotor.setPower(chassisBackLeftPow);
            frontRightMotor.setPower(chassisFrontRightPow);
            backRightMotor.setPower(chassisBackRightPow);
            intakeMotor.setPower(intakeExtendMotorPow);
            outakeRightMotor.setPower(outtakeExtendMotorPow);
            outakeLeftMotor.setPower(outtakeExtendMotorPow);
            intakeSpinMotor.setPower(intakeSpinMotorPow);




            //tel.addData("intakeSliderState",intakeState);
            //tel.addData("intakeCabinState",intakeCabinState);
            //tel.addData("outtakeState",outtakeState);
            //tel.addData("outakeArmServoPOS GO TO", outtakePivotServoPos);
            //tel.addData("outakeSamplePOS GO TO ", outtakeClawServoPos);
            //tel.addData("intakeRotateServoPosition", intakePivotServoPos);
            //tel.addData("intakeExtendMotorPow",intakeExtendMotorPow);
            //tel.addData("intake current pos",intakeMotor.getCurrentPosition());
            //tel.addData("outakeMotorPow",outtakeExtendMotorPow);
            //tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            //tel.addData("outtake current pos",outakeLeftMotor.getCurrentPosition());

            tel.addData("color stuff",currentStateOfSampleInIntake);
            tel.addData("blue color",colors.blue);
            tel.addData("red color",colors.red);
            tel.addData("is color not used", isColorSensorNotInUse);
            tel.addData("curent team color",currentTeam);
            tel.addData("Outtake Target Pos Adder",outtakeTargetPosAdder);
            tel.addData("is holding", holdMode);
            tel.addData("is braking", brakeMode);

            tel.addData("current time",System.nanoTime());
            tel.addData("time diference",System.nanoTime() - current_time);
            current_time = System.nanoTime();

            tel.update();
        }

        telemetryThreadRunning = false;
        telemetryThread.join(); // wait for the thread to finish

    }

}