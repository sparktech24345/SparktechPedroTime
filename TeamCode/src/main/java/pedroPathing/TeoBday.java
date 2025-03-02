package pedroPathing;


import static pedroPathing.PositionStorage.*;
import static pedroPathing.Toggle.toggle_var;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.States.IntakeFSM;
import pedroPathing.States.IntakeStateExtendedHM;
import pedroPathing.States.IntakeStateExtendedRo2v2;
import pedroPathing.States.IntakeStateRetractedForNoTransfer;
import pedroPathing.States.IntakeStateRetractedRo2;
import pedroPathing.States.IntakeStateWallPURetraction;
import pedroPathing.States.IntakeStateWallPURetractionRo2v2;
import pedroPathing.States.OutakeHMandWallPU;
import pedroPathing.States.OuttakeFSM;
import pedroPathing.States.OuttakeSpecimenHang;
import pedroPathing.States.OuttakeStateBasket;
import pedroPathing.States.OuttakeStateSamplePickUp;
import pedroPathing.States.OuttakeStateSpecimen;
import pedroPathing.States.OuttakeStateStandbyDownWithSample;
import pedroPathing.States.OuttakeStateStandbyWithSampleUp;
import pedroPathing.States.OuttakeStateTranfer;
import pedroPathing.tests.Config;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "La multi ani Teo!!", group = "Linear OpMode")
public class TeoBday extends LinearOpMode {

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        PositionStorage.resetStuff();

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
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/

        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");
        //declare servos
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Config.configureOtos(telemetry, myOtos);
        //Thread
        //TODO maybe start the Thread        multiThread.start();

        // Initialize Outtake states
        OuttakeStateSpecimen outtakeSpecimen = new OuttakeStateSpecimen();
        OuttakeSpecimenHang outtakeSpecimenHang = new OuttakeSpecimenHang();
        OuttakeStateBasket outtakeBasket = new OuttakeStateBasket();
        OuttakeStateSamplePickUp outtakeSamplePickUp = new OuttakeStateSamplePickUp();
        OuttakeStateStandbyDownWithSample outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
        OuttakeStateTranfer outtakeStateTranfer = new OuttakeStateTranfer();
        OuttakeStateStandbyWithSampleUp outtakeStandbyUp = new OuttakeStateStandbyWithSampleUp();
        OutakeHMandWallPU outakeHMandWallPU = new OutakeHMandWallPU();

        // Initialize Intake states
        IntakeStateRetractedRo2 intakeRetractedRo2 = new IntakeStateRetractedRo2();
        IntakeStateExtendedRo2v2 intakeExtendedRo2v2 = new IntakeStateExtendedRo2v2();
        IntakeStateExtendedHM intakeExtendedRo2v2HM = new IntakeStateExtendedHM();
        IntakeStateWallPURetraction intakeStateWallPURetraction = new IntakeStateWallPURetraction();
        IntakeStateWallPURetractionRo2v2 intakeStateWallPURetractionHMRo2v2 = new IntakeStateWallPURetractionRo2v2();
        IntakeStateRetractedForNoTransfer intakeStateRetractedForNoTransfer= new IntakeStateRetractedForNoTransfer();

        // Create the Outtake FSM with the initial state
        OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStandbyDown);
        outtakeFSM.executeCurrentState();

        // Create the Intake FSM with the initial state
        IntakeFSM intakeFSM = new IntakeFSM(intakeRetractedRo2);
        intakeFSM.executeCurrentState();

        teoBdayTimer = System.currentTimeMillis();

        intakeRotateServo.setPosition(intakeRotateServoPosition / 360);
        outakeArmServo.setPosition(outakeArmServoPosition / 360);
        outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
        isOuttakeStateStandbyWithSample = false;
        Config.configureOtos(telemetry, myOtos);

        waitForStart();

        if (isStopRequested()){
            //multiRunnable.stopRunning();
            return;
        }

        while (opModeIsActive()) {

            //temp
            if(teoBdayTimer < System.currentTimeMillis()+8000){
                teoBdayTimer = System.currentTimeMillis();
                teoBdayCase++;
            }





            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            boolean slowdown = gamepad1.left_bumper;

            ///gamepad2
            double intakeinput = gamepad2.left_stick_y;
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);


            // Slowy downy
            double slowydowny = 3;
            double slowyDownyManal = 3;

            // reversing values
            pivot = -pivot;

            //Arm pos
            armServoPos = outakeArmServo.getPosition() * 360;

            //increment
            PIDincrement = 1;


            //Selectare Echipa
            if (gamepad2.left_bumper && gamepad2.start)
                team = "BLUE";
            if (gamepad2.right_bumper && gamepad2.start)
                team = "RED";

            //just in case
            if (gamepad1.left_bumper && gamepad1.start)
                team = "BLUE";
            if (gamepad1.right_bumper && gamepad1.start)
                team = "RED";


            //Declare colors and team output
            //telemetry.addData("Curent team:", team);
            String color = "";
            if (colors.red > colors.blue && colors.red > colors.green)
                color = "BLUE"; //ACTUALLY RED BUT NEEDS INVERSION TO SPIT OUT
            if (colors.blue > colors.red && colors.blue > colors.green)
                color = "RED";  //ACTUALLY BLUE BUT NEEDS INVERSION TO SPIT OUT
            if (colors.green > colors.blue && colors.green > colors.red)
                color = "YELLOW";


            //If bad sample then kick out
            if ((colors.red >= 0.0015 || colors.blue >= 0.0015) && !transferDisabled) {
                if (color.equals(team)) {
                    wasBadSample = true;
                } else{
                    if (wasIntakeStateExtended) {
                        colortimer = System.currentTimeMillis();
                        wasIntakeStateExtended = false;
                        outtakeFSM.setState(outtakeStateTranfer);
                        outtakeFSM.executeCurrentState();
                        shouldTransfer = true;
                        transferTimerInit = true;
                        HeadUpIntake = true;
                    }
                }
            }


            ///BEGINNING STATES CODE


            //Intake Button Function for 1A
            if (gamepad1.a)
                isPressedA1 = true;
            if (!gamepad1.a && isPressedA1) {
                if (intakeFSM.currentStateIntake != intakeExtendedRo2v2) {
                    //code state 2 ( probabil restras)
                    intakeFSM.setState(intakeExtendedRo2v2);
                    intakeFSM.executeCurrentState();
                    takeWhileDisabled = true;
                }
                else  if (intakeFSM.currentStateIntake != outtakeStateTranfer && intakeFSM.currentStateIntake != intakeRetractedRo2) {
                    //code state 1    //was Modified!!!!
                    //outtakeFSM.setState(outtakeStateTranfer); without color sensor
                    //outtakeFSM.executeCurrentState();
                    outtakeFSM.setState(outtakeStateTranfer);
                    outtakeFSM.executeCurrentState();
                    intakeFSM.setState(intakeRetractedRo2);
                    intakeFSM.executeCurrentState();
                } else telemetryOhNo = true;

                isPressedA1 = false;
            }

            //TEOOOOO TESTEAZA bs
            //TRANSFER

            //Disable transfer
            if(gamepad2.a)
                isPressedA2 = true;
            if(isPressedA2 && !gamepad2.a){
                transferDisabled = !transferDisabled;
                isPressedA2 = false;
            }



            //TRANSFER INIT, big if time
            if (((outakeArmServo.getPosition()*360 <= outtakeArmServoPosAtRo2v2TransferPickUp + 5))
                    && outtakeFSM.currentStateOutake == outtakeStateTranfer
                    && (colors.red >= 0.0015 || colors.blue >=0.0015)
                    && !transferDisabled
                    || gamepad1.right_trigger>=0.4
                    //&& intakeRotateServo.getPosition()*360<=65
            ) {

                //start timer
                if(wasBambuExtended) {
                    bambuTransferTimer = System.currentTimeMillis();
                    wasBambuExtended = false;
                    doOnceyTransfer = true;
                    someExtraThingDoOnce = true;
                }

                //ACTUAL TRANSFER
                //claw down
                if (bambuTransferTimer + 200 < System.currentTimeMillis()) {
                    if(doOnceyTransfer) intakeRotateServoPosition = intakeRo2SmashPos;
                    if(doOnceyTransfer &&  bambuTransferTimer + 600 < System.currentTimeMillis()){
                        intakeTargetPos =  intakeSlidersRo2Transfer + 60;
                        doOnceyTransfer = false;
                        intakeExtraSpinOUTPUTTimer = System.currentTimeMillis();
                        intakeExtraSpinOUTPUTDoOnce = true;
                        //intakeMotorPickUpPower = -0.55;
                        DontDoTransferBeforeTransfer = true;
                        someOtherBollean = true;
                    }
                    if(someOtherBollean && bambuTransferTimer + 800 < System.currentTimeMillis()){
                        intakeFSM.setState(intakeRetractedRo2);
                        intakeFSM.executeCurrentState();
                        intakeTargetPos += intakeTransferSlidersAdder;
                        intakeTransferSlidersAdder = 0;
                        someOtherBollean = false;
                    }
                    if((intakeMotor.getCurrentPosition() <= intakeTargetPosAdder + intakeTargetPos +14)&& someExtraThingDoOnce&&  bambuTransferTimer + 800 + addedTimer < System.currentTimeMillis()) {
                        noWiglyTransferTimer = System.currentTimeMillis();
                        noWiglyPls = true;
                        someExtraThingDoOnce = false;
                    }
                }
                //close thingyy
                if(noWiglyPls && noWiglyTransferTimer + 200 < System.currentTimeMillis()){
                    outakeSampleServoPosition = outakeSampleRetracted;
                }
                //transfer done
                if(noWiglyPls && noWiglyTransferTimer + 500 < System.currentTimeMillis() && DontDoTransferBeforeTransfer){
                    outtakeFSM.setState(outtakeStandbyDown);
                    outtakeFSM.executeCurrentState();
                    outakeSampleServoPosition = outakeSampleRetracted;
                    //intakeRotateServoPosition = intakeRotateAfterRo2Trasfer;
                    extendABitAfterRo2Transfer = true;
                    intakeShouldRetractAfterTransfer = true;
                    noWiglyPls = false;
                    DontDoTransferBeforeTransfer = false;
                }
            }
            if(extendABitAfterRo2Transfer && !transferDisabled){
                intakeTargetPos = extendABitAfterRo2TransferPos;
                extendABitAfterRo2Transfer = false;
            }
            if(intakeShouldRetractAfterTransfer && outakeArmServo.getPosition()*360 >= outakeArmTransferPos - 10 && !transferDisabled){
                intakeShouldRetractAfterTransfer = false;
                intakeShouldRetractAfterTransferTimerToggle = true;
                intakeShouldRetractAfterTransferTimer = System.currentTimeMillis();
            }
            if(intakeShouldRetractAfterTransferTimerToggle && intakeShouldRetractAfterTransferTimer + 400 < System.currentTimeMillis()&& !((colors.red >= 0.0015 || colors.blue >=0.0015) ) && !transferDisabled){
                intakeFSM.setState(intakeRetractedRo2);
                intakeFSM.executeCurrentState();
                intakeShouldRetractAfterTransferTimerToggle = false;
            }
            //risky




            //Outtake Basket
            if (gamepad1.x)
                isPressedX1 = true;
            if (!gamepad1.x && isPressedX1) {
                if (outtakeFSM.currentStateOutake != outtakeBasket) {
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeBasket) {
                    outtakeFSM.setState(outtakeSamplePickUp);
                    outtakeFSM.executeCurrentState();
                    isHeldBascket = false;
                } else telemetryOhNo = true;
                isPressedX1 = false;
            }
            if(gamepad1.x && outtakeFSM.currentStateOutake == outtakeBasket){
                outakeSampleServoPosition = servoextended;
            }

            //Outtake Specimen
            if (gamepad1.b)
                isPressedB1 = true;
            if (!gamepad1.b && isPressedB1) {
                if (outtakeFSM.currentStateOutake != outtakeSpecimen && outtakeFSM.currentStateOutake != outtakeSpecimenHang) {
                    outtakeFSM.setState(outtakeSpecimen);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeSpecimen) {
                    outtakeFSM.setState(outtakeSpecimenHang);
                    outtakeFSM.executeCurrentState();
                } else if (outtakeFSM.currentStateOutake == outtakeSpecimenHang) {
                    outakeSampleServoPosition = servoextended;
                    goToPickUp = true;
                    startingTimer5 = System.currentTimeMillis();
                } else telemetryOhNo = true;
                isPressedB1 = false;
            }
            if (goToPickUp && startingTimer5 + afterSpecimenOpenTime < System.currentTimeMillis()) {
                outtakeFSM.setState(outtakeSamplePickUp);
                outtakeFSM.executeCurrentState();
                goToPickUp = false;
            }





            //Intake positions
            if (gamepad2.dpad_left) {
                intakeTargetPos = 510;
                gravityAdder = 1;
                intakeTransferSlidersAdder=0;
            } // 4/4
            if (gamepad2.dpad_down) {
                intakeTargetPos = 377;
                gravityAdder = 1;
                intakeTransferSlidersAdder=0;
            } // 3/4
            if (gamepad2.dpad_right){
                intakeTargetPos = 245; // 2/4
                gravityAdder = 1;
                addedTimer = 500;
                intakeTransferSlidersAdder=-20;
            }
            if(gamepad2.dpad_up) {
                intakeTargetPos = 112;  // 1/4
                gravityAdder = 1;
                addedTimer = 500;
                intakeTransferSlidersAdder=-30;
            }
            if(gamepad2.left_bumper)
                intakeTargetPos = 0;





            //Outake HM
            if(gamepad1.y){
                isPressedY1 = true;
            }
            if(!gamepad1.y && isPressedY1){
                if(outtakeFSM.currentStateOutake != outakeHMandWallPU) {
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();
                    wasOutputHM2 = true;
                    intakeRotateServoPosition = intakeRotateForWallPickUp;
                    timerSticlaDeApa = System.currentTimeMillis();
                }
                else if(outtakeFSM.currentStateOutake == outakeHMandWallPU){
                    startingTimer2=System.currentTimeMillis();
                    wasOutputHM = true;
                    outakeSampleServoPosition = outakeSampleRetracted;
                }
                else telemetryOhNo = true;
                isPressedY1 = false;
            }
            if(wasOutputHM && System.currentTimeMillis() > startingTimer2 + 200 ){
                outtakeFSM.setState(outtakeStandbyUp);
                outtakeFSM.executeCurrentState();
                wasOutputHM = false;
            }
            if(wasOutputHM2 && System.currentTimeMillis() > timerSticlaDeApa + 300 ){
                intakeFSM.setState(intakeStateWallPURetractionHMRo2v2);
                intakeFSM.executeCurrentState();
                wasOutputHM2 = false;
            }
            ///END OF STATES CODE





            //OutakeIncrement
            if(gamepad1.dpad_up)
                outakeTargetPosAdder += 2;
            if(gamepad1.dpad_down)
                outakeTargetPosAdder -= 2;




            //Intake Increment
            if (gamepad2.right_trigger > 0.4 && intakeTargetPos <= 270)
                intakeTargetPosAdder += 2;
            if (gamepad2.left_trigger > 0.4 && intakeTargetPos >= 0)
                intakeTargetPosAdder -= 2;





            //Hang code
            if(gamepad1.dpad_left) isPressedDL1 = true;
            if(!gamepad1.dpad_left && isPressedDL1){
                if(!hangTime) outakeTargetPos = -2800;
                else if(hangTime) outakeTargetPos = -1550;
                hangTime = !hangTime;
                isPressedDL1 = false;
            }






            //Outputing Samples if Nedded
            if(intakeMotorPickUpPower != -0.55)
                rememberPosOfServoOut = intakeMotorPickUpPower;
            if((Toggle.outputtoggle(gamepad1.right_bumper || wasBadSample)!=0) || isOutputinHM){
                intakeMotorPickUpPower =-0.55;
                wasBadSample = false;
            }
            else intakeMotorPickUpPower = rememberPosOfServoOut;


            /*//Outaputing samples with gamepad 2 for HM
            if(intakeMotorPickUpPower != -0.55 && intakeMotorPickUpPower != -0.55)
                rememberPosOfServoOut = intakeMotorPickUpPower;
            if(Toggle.outputtoggle2(isOutputinHM)!=0){
                intakeMotorPickUpPower =-0.55;
                isOutputinHM = false;
            }
            else intakeMotorPickUpPower = rememberPosOfServoOut;//*/



            //extra spin after taking
            if(intakeExtraSpinTimer + 25 < System.currentTimeMillis() && intakeExtraSpinDoOnce){
                if(intakeMotorPickUpPower>0)
                    intakeMotorPickUpPower = 0;
                intakeExtraSpinDoOnce = false;
            }



            //GAMEPAD2 RELEASE TRIGGER
            if(gamepad2.y)
                isPressedY2 = true;
            if (!gamepad2.y && isPressedY2) {
                if (outakeSampleServoPosition != servoextended)
                    outakeSampleServoPosition = servoextended;
                else if (outakeSampleServoPosition == servoextended)
                    outakeSampleServoPosition = 10;
                else telemetryOhNo = true;
                isPressedY2 = false;
            }



            //Sample Output for HM organized
            if(gamepad2.b)
                isPressedB2 = true;
            if(isPressedB2 && !gamepad2.b){
                if(isOutputinHM){ isOutputinHM = false;}
                else {
                    intakeFSM.setState(intakeExtendedRo2v2HM);
                    intakeFSM.executeCurrentState();
                    SpitOutSampleHMTimer = System.currentTimeMillis();
                    SpitOutSampleHM = true;
                    SpitOutSampleHM2 = true;
                    isOutputting = true;
                }
                isPressedB2 = false;
            }

            if(SpitOutSampleHM2 && intakeFSM.currentStateIntake == intakeExtendedRo2v2HM && SpitOutSampleHMTimer + 450 < System.currentTimeMillis()){
                //intakeMotorPickUpPower = -0.55;
                SpitOutSampleHM2 = false;
                isOutputinHM = true;
            }


            if(intakeFSM.currentStateIntake == intakeExtendedRo2v2HM && SpitOutSampleHM && SpitOutSampleHMTimer + 650 < System.currentTimeMillis() && !isOutputinHM){
                intakeFSM.setState(intakeStateRetractedForNoTransfer);
                intakeFSM.executeCurrentState();
                outtakeFSM.setState(outtakeStandbyDown);
                outtakeFSM.executeCurrentState();
                isOutputting = false;
                SpitOutSampleHM = false;
                isOutputinHM = false;
                intakeMotorPickUpPower =0;
            }





            //Intake target position
            if (intakeinput < 0)
                intakeTargetPos += 15;
            if (intakeinput > 0)
                intakeTargetPos -= 15;




            //retract for no transfer mode for specimen
            if(intakeFSM.currentStateIntake != intakeExtendedRo2v2HM && !isOutputting && transferDisabled &&  takeWhileDisabled  &&  (colors.red >= 0.0015 || colors.blue >=0.0015)){
                if (color.equals(team) || color.equals("YELLOW")) {
                    wasBadSample = true;
                }
                else {
                    intakeFSM.setState(intakeStateRetractedForNoTransfer);
                    intakeFSM.executeCurrentState();
                    takeWhileDisabled = false;
                }
            }



            //PID STUFF
            double intakeMotorPower = 0;
            intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            double outakeMotorPower;
            outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outakeMotorPower *= PIDincrement;



            //calculating nedded power by method 1
            frontRightPowerCat = (pivot - vertical - horizontal);
            backRightPowerCat = (pivot - vertical + horizontal);
            frontLeftPowerCat = (pivot + vertical - horizontal);
            backLeftPowerCat = (pivot + vertical + horizontal);


            //TELEMETRY

            //telemetry.addData("intakerotatey",intakeRotateServo.getPosition());
            //telemetry.addData("intakerotateygoalposition",intakeRotateServoPower);
            //telemetry, again
            /*telemetry.addData("IMU angular velocity",imu.getRobotAngularVelocity(AngleUnit.DEGREES));
            telemetry.addData("getRobotOrientationAsQuaternion",imu.getRobotOrientationAsQuaternion());
            telemetry.addData("getRobotYawPitchRollAngles",imu.getRobotYawPitchRollAngles());//*/
            /*telemetry.addData("outake Error", outakeControlMotor.getLastError());
            telemetry.addData("curent Pos RIGHT", outakeRightMotor.getCurrentPosition());
            telemetry.addData("target Pos", outakeTargetPos);
            telemetry.addData("powah", outakeMotorPower);
            telemetry.addData("OutakeAdder", outakeTargetPosAdder);
            telemetry.addData("spinytimerstart", spinyOutputToggle);
            telemetry.addData("intakeCurentPOs", intakeMotor.getCurrentPosition());
            telemetry.addData("intakeTargetPos", intakeTargetPos);
            telemetry.addData("intakepower", intakeMotorPower);//*/
            //telemetry.addData("intake rotate", intakeRotateServo.getPosition()*360);
            //telemetry.addData("intake Adder", intakeTargetPosAdder);
            //*/
            //telemetry.addData("servo POs", tester.getPosition());
            //telemetry.addData("y", pos.y);
            //telemetry.addData("x", pos.x);
            //telemetry.addData("rotation/orientation", pos.h);
            //telemetry.addData("outakeArmServoCurrentPosition",outakeArmServo.getPosition());
            //telemetry.addData("outakeArmServo",outakeArmServoPosition);
            //telemetry.addData("outakeSampleServoCurrentPosition",outakeSampleServo.getPosition());
            //telemetry.addData("outakeSampleServo",outakeSampleServoPosition);


            //Automatic slow down
            if ((outtakeFSM.currentStateOutake == outtakeBasket
                    //|| outtakeFSM.currentStateOutake == outtakeSpecimen
                    //|| outtakeFSM.currentStateOutake == outtakeSpecimenHang
                    || outtakeFSM.currentStateOutake == outakeHMandWallPU
                    || intakeFSM.currentStateIntake  == intakeExtendedRo2v2)
                    && !toggle_var
            ) {
                frontLeftPowerCat /= slowydowny;
                backRightPowerCat /= slowydowny;
                frontRightPowerCat /= slowydowny;
                backLeftPowerCat /= slowydowny;
            }//*/


            //Manual Slowdown Function
            if(Toggle.FirsToggle(slowdown)){
                frontLeftPowerCat /= slowyDownyManal;
                backRightPowerCat /= slowyDownyManal;
                frontRightPowerCat /= slowyDownyManal;
                backLeftPowerCat /= slowyDownyManal;
            }



            // set motor power
            frontLeftMotor.setPower(frontLeftPowerCat);
            backLeftMotor.setPower(backLeftPowerCat);
            frontRightMotor.setPower(frontRightPowerCat);
            backRightMotor.setPower(backRightPowerCat);
            intakeMotor.setPower(intakeMotorPower);
            outakeRightMotor.setPower(outakeMotorPower);
            outakeLeftMotor.setPower(outakeMotorPower);//*/
            intakeSpinMotor.setPower(intakeMotorPickUpPower);



            //Set servo Positions
            intakeRotateServo.setPosition((intakeRotateServoPosition+gravityAdder) / 360);
            outakeArmServo.setPosition(outakeArmServoPosition / 360);
            outakeSampleServo.setPosition(outakeSampleServoPosition / 360);

            //telemetry.addData("color",color);
            //telemetry.addData("IntakeFsm",stateStringIntake);
            //telemetry.addData("OutakeFsm",stateStringOutake);
            telemetry.addData("La multi ani Teo!!!","");
            telemetry.addData(TeoBdayString(teoBdayCase),"");
            if(telemetryOhNo)
                telemetry.addData("OH NOOOO",true);
            updateTelemetry(telemetry);
        }
        //multiRunnable.stopRunning();
    }

    public String TeoBdayString(int cas){
        String outString = "La multi ani Teoooo";
        switch (cas){
            case 1:
                outString = "Buna Teeeoooo, sper ca citesti asta";
                break;

            case 2:
                outString = "La multi ani si tot ce iti doresti!";
                break;

            case 3:
                outString = "sper sa ai parte de cei mai buni prieteni si experiente acum de ziua ta";
                break;

            case 4:
                outString = "iti urez tot ce e ma bun si iti garantez ca";
                break;

            case 5:
                outString = "de acum in colo totul va fi din ce in ce mai bine";
                break;

            case 6:
                outString = "sper ca am fost un prieten cat mai bun pentru tine";
                break;

            case 7:
                outString = "mai ajutat si m-ai facut sa rad la momentele de bine si de rau";
                break;

            case 8:
                outString = "ai fost mereu acolo pentru mine si ma bucur ca am avut atunci ncredere sa incep conversatia";
                break;

            case 9:
                outString = "esti un prieten foarte bun si iti multumesc pentru tot ce ai facut pentru mine";
                break;

            case 10:
                outString = "promit sa raman mereu alaturi de tine si sa te ajut la orice ocazie";
                break;

            case 11:
                outString = "dinou, la multi ani Teo!, la multi ani prietenului meu!";
                break;

            default:
                outString = "La multi ani Teoooo";
                break;
        }
        return outString;
    }

}