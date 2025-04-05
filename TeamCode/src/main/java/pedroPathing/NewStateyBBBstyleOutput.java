package pedroPathing;


import static pedroPathing.OrganizedPositionStorage.*;
import static pedroPathing.ClassWithStates.*;
import static pedroPathing.newOld.PositionStorage.addedTimer;
import static pedroPathing.newOld.PositionStorage.gravityAdder;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.intakeTransferSlidersAdder;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.newOld.PositionStorage;
import pedroPathing.tests.Config;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "BBBNewStatesOutput", group = "Linear OpMode")
public class NewStateyBBBstyleOutput extends LinearOpMode {

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

        Config.configureOtos(telemetry, myOtos);


        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos+gravityAdder) / 360);
        outakeArmServo.setPosition(outtakePivotServoPos / 360);
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


            currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);



            ///CONTROLS

            //PICK UP
            if(gamepad1.a) isPressedA1 = true;
            if(!gamepad1.a && isPressedA1){
                if(!(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting || intakeCabinState == intakeCabinStates.intakeCabinDownOutputting)) {
                    intakeCabinDownCollecting();
                    outtakeTransfer();
                    isAfterIntakeBeenDownColecting = true;
                }
                else{
                    intakeRetracted();
                    intakeCabinTransferPosition();
                    outtakeTransfer();
                    isAfterIntakeBeenDownColecting = false;
                }
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
            }
            if(isAfterOuttakeClosedClawAtWallSpecimen && outtakeAfterHasClosedClawAtWallSpecimenTimer + 50 < System.currentTimeMillis()){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeSpecimenHang();
            }
            if(isAfterOuttakeScoredSpecimen && outtakeSpecimenAfterScoreTimer + 50 < System.currentTimeMillis()){
                outtakeWallPickUpNew();
                isAfterOuttakeScoredSpecimen = false;
            }



            //BASKET SCORING
            if(gamepad1.x) isPressedX1 = true;
            if(!gamepad1.x && isPressedX1){
                if(!(outtakeState == outtakeStates.outtakeBasket)){
                    intakeRetracted();
                    intakeCabinFullInBot();
                    outtakeBasket();
                }
                else{
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    outtakeAfterBasketSampleScoreTimer = System.currentTimeMillis();
                    isAfterOuttakeScoredBasketSample = true;
                }
            }
            if(isAfterOuttakeScoredBasketSample && outtakeAfterBasketSampleScoreTimer + 50 < System.currentTimeMillis()) {
                outtakePivotServoPos = outtakePivotServoTransferPos;
                if(outtakeAfterBasketSampleScoreTimer + 300 < System.currentTimeMillis()) {
                    outtakeTransfer();
                    isAfterOuttakeScoredBasketSample = false;
                }
            }



            //WALL PICK UP
            if(gamepad1.y) isPressedY1 = true;
            if(!gamepad1.y && isPressedY1){
                intakeRetracted();
                intakeCabinFullInBot();
                outtakeWallPickUpNew();
            }


            //weird stuff for hm outputting
            if(     gamepad2.b &&
                    (intakeCabinState == intakeCabinStates.intakeCabinFullInBot ||
                    intakeCabinState == intakeCabinStates.intakeCabinTransferPosition ||
                    intakeCabinState == intakeCabinStates.intakeCabinFullInBotOutputting)
            ){
                intakeCabinFullInBotOutputting();
                isAfterBotHasBeenOutputting = true;
            }
            else if(isAfterBotHasBeenOutputting){
                intakeCabinFullInBot();
                isAfterBotHasBeenOutputting = false;
            }




            ///SOME STUFF


            //auto retract
            if(currentStateOfSampleInIntake == colorSensorOutty.correctSample && isAfterIntakeBeenDownColecting){
                intakeRetracted();
                intakeCabinTransferPosition();
                outtakeTransfer();
                isAfterIntakeBeenDownColecting = false;
            }

            //auto eject
            if(currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                intakeCabinDownOutputting();
                isIntakeOutputting = true;
                intakeOutputtingTimer = System.currentTimeMillis();
            }
            if(isIntakeOutputting && intakeOutputtingTimer + 150 < System.currentTimeMillis()){
                intakeCabinDownCollecting();
                isIntakeOutputting = false;
            }



            //manual eject
            if(gamepad1.left_bumper){
                intakeSpinMotorPow = -0.55;
                isIntakeOutputting = true;
                intakeOutputtingTimer = System.currentTimeMillis();
            }
            if(isIntakeOutputting && intakeOutputtingTimer + 150 < System.currentTimeMillis()){
                if(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting) intakeSpinMotorPow = 1;
                else if(intakeCabinState == intakeCabinStates.intakeCabinDownOutputting) intakeSpinMotorPow = -0.55;
                else intakeSpinMotorPow = 0;
                isIntakeOutputting = false;
            }


            //chosing intake positions
            //Intake positions
            if (gamepad2.dpad_left)  intakeExtended4out4();
            if (gamepad2.dpad_down)  intakeExtended3out4();
            if (gamepad2.dpad_right) intakeExtended2out4();
            if(gamepad2.dpad_up)     intakeExtended1out4();
            if(gamepad2.left_bumper) intakeRetracted();








            //PIDs
            double intakeMotorPower = 0;
            intakeMotorPower = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            double outakeMotorPower;
            outakeMotorPower = outakeControlMotor.PIDControlUppy(outtakeExtendMotorTargetPos-outtakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outakeMotorPower *= PIDincrement;




            chassisFrontRightPow = (pivot - vertical - horizontal);
            chassisBackRightPow = (pivot - vertical + horizontal);
            chassisFrontLeftPow = (pivot + vertical - horizontal);
            chassisBackLeftPow = (pivot + vertical + horizontal);


            //slowdown
            double slowyDownyManal = 2.5;
            double slowyDownyAuto = 1.75;

            //manual slowdown
            if(gamepad1.left_bumper){
                chassisFrontLeftPow /= slowyDownyManal;
                chassisBackRightPow /= slowyDownyManal;
                chassisFrontRightPow /= slowyDownyManal;
                chassisBackLeftPow /= slowyDownyManal;
            }
            //auto slowdown
            else if(outtakeState == outtakeStates.outtakeBasket ||
                    outtakeState == outtakeStates.outtakeWallPickUpNew ||
                    outtakeState == outtakeStates.outtakeSpecimenHang){
                chassisFrontLeftPow /= slowyDownyAuto;
                chassisBackRightPow /= slowyDownyAuto;
                chassisFrontRightPow /= slowyDownyAuto;
                chassisBackLeftPow /= slowyDownyAuto;
            }





            // set motor power
            frontLeftMotor.setPower(chassisFrontLeftPow);
            backLeftMotor.setPower(chassisBackLeftPow);
            frontRightMotor.setPower(chassisFrontRightPow);
            backRightMotor.setPower(chassisBackRightPow);
            intakeMotor.setPower(intakeExtendMotorPow);
            outakeRightMotor.setPower(outtakeExtendMotorPow);
            outakeLeftMotor.setPower(outtakeExtendMotorPow);
            intakeSpinMotor.setPower(intakeSpinMotorPow);


            //Set servo Positions
            intakeRotateServo.setPosition((intakePivotServoPos+gravityAdder) / 360);
            outakeArmServo.setPosition(outtakePivotServoPos / 360);
            outakeSampleServo.setPosition(outtakeClawServoPos / 360);


            updateTelemetry(telemetry);
        }

    }

}