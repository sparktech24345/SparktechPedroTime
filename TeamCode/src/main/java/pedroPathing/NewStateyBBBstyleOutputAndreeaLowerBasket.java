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
import static pedroPathing.ClassWithStates.outtakeLowerBasket;
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
import static pedroPathing.OrganizedPositionStorage.isPressedX1;
import static pedroPathing.OrganizedPositionStorage.isPressedX2;
import static pedroPathing.OrganizedPositionStorage.isPressedY2;
import static pedroPathing.OrganizedPositionStorage.isTimeToRefreshOutptingTime;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import pedroPathing.tests.Config;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "BBBNewStatesOutputA", group = "Linear OpMode")
@Disabled
public class NewStateyBBBstyleOutputAndreeaLowerBasket extends LinearOpMode {

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;
    MultipleTelemetry tel;

    @Override
    public void runOpMode() throws InterruptedException {

        OrganizedPositionStorage.resetStuff();

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");


        //declare servos
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Config.configureOtos(tel, myOtos);


        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);



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
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            //Selectare Echipa
            if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
                currentTeam = colorList.blue;
            if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
                currentTeam = colorList.red;

            if(Toggle.FirsToggle(gamepad1.left_trigger >= 0.4 && gamepad1.right_trigger >=0.4)){
                horizontal = - horizontal;
                vertical = - vertical;
            }


            currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);



            ///CONTROLS

            //PICK UP
            /*
            double timeAtTransfer = 0;
            boolean isYetToGrab = false;
            */
            if(gamepad1.a) isPressedA1 = true;
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
            if(gamepad1.x) isPressedX1 = true;
            if(!gamepad1.x && isPressedX1){
                if(!(outtakeState == outtakeStates.outtakeBasket)){

                    isAfterOuttakeClawClosedAfterTransfer = true;
                    intakeAfterTransferClosedClawTimer = System.currentTimeMillis();
                    isAtStateOfLettingBasketSampleGo = true;
                }
                else intakeRetracted();
                isPressedX1 = false;
            }
            if(isAfterOuttakeClawClosedAfterTransfer && intakeAfterTransferClosedClawTimer + 300 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinTransferPosition();
                outtakeBasket();
                isAfterOuttakeClawClosedAfterTransfer = false;
            }
            //going down after, quite complicated cuz holding to let sample go
            if(isAtStateOfLettingBasketSampleGo && gamepad1.x){
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
                if(!(outtakeState == outtakeStates.outtakeLowerBasket)){
                    isAfterOuttakeClawClosedAfterTransfer = true;
                    intakeAfterTransferClosedClawTimer = System.currentTimeMillis();
                    isAtStateOfLettingBasketSampleGo = true;
                }
                else intakeRetracted();
                isPressedB2 = false;
            }
            if(isAfterOuttakeClawClosedAfterTransfer && intakeAfterTransferClosedClawTimer + 300 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinTransferPosition();
                outtakeLowerBasket();
                isAfterOuttakeClawClosedAfterTransfer = false;
            }
            //going down after, quite complicated cuz holding to let sample go
            if(isAtStateOfLettingBasketSampleGo && gamepad2.b){
                outtakeClawServoPos = outtakeClawServoExtendedPos;
                isAfterOuttakeScoredBasketSample = true;
                isAtStateOfLettingBasketSampleGo = false;
                outtakeAfterBasketSampleScoreTimer = System.currentTimeMillis();
            }
            if(!gamepad2.b && isAfterOuttakeScoredBasketSample) {
                outtakePivotServoPos = outtakePivotServoTransferPos;
                if(outtakeAfterBasketSampleScoreTimer + 300 < System.currentTimeMillis()) {
                    outtakeTransfer();
                    isAfterOuttakeScoredBasketSample = false;
                }
            }
            //some weird bug that can be easy fix
            if(outtakeExtendMotorTargetPos == 0 && outtakeState == outtakeStates.outtakeLowerBasket){
                outtakeLowerBasket();
            }
            //*/







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
            if((currentStateOfSampleInIntake == colorSensorOutty.correctSample && isAfterIntakeBeenDownColecting) ||
                shouldAutoCollect
            ){
                shouldAutoCollect = false;
                intakeRetracted();
                //makins sure sample enetered the intake fully with a small timer
                isAfterIntakeBeenDownColecting = false;
                isIntakeSpinMOtorAfterJustTaking = true;
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

                basketStandbyState = 0;
                //outtakeExtendMotorTargetPos = outtakeMotorStandByPos;

            }

            //smol give out after the extra take in


            if(shouldStopIntakeCabinSpinningAfterTakig && shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis()){
                intakeSpinMotorPow = -0.8;
                shouldStopIntakeCabinSpinningAfterTakig = false;
                hasSmolOutputed = true;
                hasSmolOutputedTimer = System.currentTimeMillis();
            }
            //and then stop the power stuff
            if(hasSmolOutputed && hasSmolOutputedTimer + 50 <System.currentTimeMillis()){
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
                intakeSpinMotorPow = -0.75;
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
                        intakeSpinMotorPow = -1;
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


            //PIDs
            PIDincrement=1;
            double intakeExtendMotorPow;
            intakeExtendMotorPow = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            if(currentStateOfSampleInIntake == colorSensorOutty.correctSample) intakeExtendMotorPow *= 1.3;
            double outtakeExtendMotorPow;
            outtakeExtendMotorPow = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos-outtakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outtakeExtendMotorPow *= PIDincrement;




            chassisFrontRightPow = (pivot - vertical - horizontal);
            chassisBackRightPow = (pivot - vertical + horizontal);
            chassisFrontLeftPow = (pivot + vertical - horizontal);
            chassisBackLeftPow = (pivot + vertical + horizontal);


            //slowdown
            double slowyDownyManal = 2.5;
            double slowyDownyAuto = 1.5;

            //manual slowdown
            if(gamepad1.right_bumper){
                chassisFrontLeftPow /= slowyDownyManal;
                chassisBackRightPow /= slowyDownyManal;
                chassisFrontRightPow /= slowyDownyManal;
                chassisBackLeftPow /= slowyDownyManal;
            }
            //auto slowdown

            else if(// intakeState == intakeStates.intakeExtended1out4 ||
            intakeState == intakeStates.intakeExtended2out4 ||
            intakeState == intakeStates.intakeExtended3out4 ||
            intakeState == intakeStates.intakeExtended4out4 )
            //intakeCabinState == intakeCabinStates.intakeCabinDownCollecting
            {
                chassisFrontLeftPow /= slowyDownyAuto;
                chassisBackRightPow /= slowyDownyAuto;
                chassisFrontRightPow /= slowyDownyAuto;
                chassisBackLeftPow /= slowyDownyAuto;
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



            // set motor power
            frontLeftMotor.setPower(chassisFrontLeftPow*0.8);
            backLeftMotor.setPower(chassisBackLeftPow*0.8);
            frontRightMotor.setPower(chassisFrontRightPow*0.8);
            backRightMotor.setPower(chassisBackRightPow*0.8);
            intakeMotor.setPower(intakeExtendMotorPow);
            outakeRightMotor.setPower(outtakeExtendMotorPow);
            outakeLeftMotor.setPower(outtakeExtendMotorPow);
            intakeSpinMotor.setPower(intakeSpinMotorPow);


            //Set servo Positions
            intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
            outakeArmServo.setPosition(outtakePivotServoPos / 328);
            outakeSampleServo.setPosition(outtakeClawServoPos / 360);


            tel.addData("intakeSliderState",intakeState);
            tel.addData("intakeCabinState",intakeCabinState);
            tel.addData("outtakeState",outtakeState);
            tel.addData("color stuff",currentStateOfSampleInIntake);
            tel.addData("outakeArmServoPOS GO TO", outtakePivotServoPos);
            tel.addData("outakeSamplePOS GO TO ", outtakeClawServoPos);
            tel.addData("intakeRotateServoPosition", intakePivotServoPos);
            tel.addData("intakeExtendMotorPow",intakeExtendMotorPow);
            tel.addData("outakeMotorPow",outtakeExtendMotorPow);
            tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            tel.addData("outtake current pos",outakeLeftMotor.getCurrentPosition());
            tel.addData("blue color",colors.blue);
            tel.addData("red color",colors.red);

            updateTelemetry(tel);
        }

    }

}