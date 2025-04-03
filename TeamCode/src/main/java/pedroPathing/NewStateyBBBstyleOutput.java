package pedroPathing;


import static pedroPathing.newOld.PositionStorage.DontDoTransferBeforeTransfer;
import static pedroPathing.newOld.PositionStorage.HeadUpIntake;
import static pedroPathing.newOld.PositionStorage.PIDincrement;
import static pedroPathing.newOld.PositionStorage.SpitOutSampleHM;
import static pedroPathing.newOld.PositionStorage.SpitOutSampleHM2;
import static pedroPathing.newOld.PositionStorage.SpitOutSampleHMTimer;
import static pedroPathing.newOld.PositionStorage.addedTimer;
import static pedroPathing.newOld.PositionStorage.afterSpecimenOpenTime;
import static pedroPathing.newOld.PositionStorage.armServoPos;
import static pedroPathing.newOld.PositionStorage.backLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.backRightPowerCat;
import static pedroPathing.newOld.PositionStorage.bambuTransferTimer;
import static pedroPathing.newOld.PositionStorage.colortimer;
import static pedroPathing.newOld.PositionStorage.doOnceyTransfer;
import static pedroPathing.newOld.PositionStorage.extendABitAfterRo2Transfer;
import static pedroPathing.newOld.PositionStorage.extendABitAfterRo2TransferPos;
import static pedroPathing.newOld.PositionStorage.frontLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.frontRightPowerCat;
import static pedroPathing.newOld.PositionStorage.goToPickUp;
import static pedroPathing.newOld.PositionStorage.gravityAdder;
import static pedroPathing.newOld.PositionStorage.hangTime;
import static pedroPathing.newOld.PositionStorage.intakeExtraSpinDoOnce;
import static pedroPathing.newOld.PositionStorage.intakeExtraSpinOUTPUTDoOnce;
import static pedroPathing.newOld.PositionStorage.intakeExtraSpinOUTPUTTimer;
import static pedroPathing.newOld.PositionStorage.intakeExtraSpinTimer;
import static pedroPathing.newOld.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.newOld.PositionStorage.intakeRo2SmashPos;
import static pedroPathing.newOld.PositionStorage.intakeRotateForWallPickUp;
import static pedroPathing.newOld.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.newOld.PositionStorage.intakeShouldRetractAfterTransfer;
import static pedroPathing.newOld.PositionStorage.intakeShouldRetractAfterTransferTimer;
import static pedroPathing.newOld.PositionStorage.intakeShouldRetractAfterTransferTimerToggle;
import static pedroPathing.newOld.PositionStorage.intakeSlidersRo2Transfer;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.intakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.intakeTransferSlidersAdder;
import static pedroPathing.newOld.PositionStorage.isHeldBascket;
import static pedroPathing.newOld.PositionStorage.isOutputinHM;
import static pedroPathing.newOld.PositionStorage.isOutputting;
import static pedroPathing.newOld.PositionStorage.isOuttakeStateStandbyWithSample;
import static pedroPathing.newOld.PositionStorage.isPressedA1;
import static pedroPathing.newOld.PositionStorage.isPressedA2;
import static pedroPathing.newOld.PositionStorage.isPressedB1;
import static pedroPathing.newOld.PositionStorage.isPressedB2;
import static pedroPathing.newOld.PositionStorage.isPressedDL1;
import static pedroPathing.newOld.PositionStorage.isPressedX1;
import static pedroPathing.newOld.PositionStorage.isPressedY1;
import static pedroPathing.newOld.PositionStorage.isPressedY2;
import static pedroPathing.newOld.PositionStorage.noWiglyPls;
import static pedroPathing.newOld.PositionStorage.noWiglyTransferTimer;
import static pedroPathing.newOld.PositionStorage.outakeArmServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeArmTransferPos;
import static pedroPathing.newOld.PositionStorage.outakeSampleRetracted;
import static pedroPathing.newOld.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.newOld.PositionStorage.outakeTargetPos;
import static pedroPathing.newOld.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.outtakeArmServoPosAtRo2v2TransferPickUp;
import static pedroPathing.newOld.PositionStorage.rememberPosOfServoOut;
import static pedroPathing.newOld.PositionStorage.servoextended;
import static pedroPathing.newOld.PositionStorage.shouldTransfer;
import static pedroPathing.newOld.PositionStorage.someExtraThingDoOnce;
import static pedroPathing.newOld.PositionStorage.someOtherBollean;
import static pedroPathing.newOld.PositionStorage.startingTimer2;
import static pedroPathing.newOld.PositionStorage.startingTimer5;
import static pedroPathing.newOld.PositionStorage.stateStringIntake;
import static pedroPathing.newOld.PositionStorage.stateStringOutake;
import static pedroPathing.newOld.PositionStorage.takeWhileDisabled;
import static pedroPathing.newOld.PositionStorage.team;
import static pedroPathing.newOld.PositionStorage.telemetryOhNo;
import static pedroPathing.newOld.PositionStorage.timerSticlaDeApa;
import static pedroPathing.newOld.PositionStorage.transferDisabled;
import static pedroPathing.newOld.PositionStorage.transferTimerInit;
import static pedroPathing.newOld.PositionStorage.wasBadSample;
import static pedroPathing.newOld.PositionStorage.wasBambuExtended;
import static pedroPathing.newOld.PositionStorage.wasIntakeStateExtended;
import static pedroPathing.newOld.PositionStorage.wasOutputHM;
import static pedroPathing.newOld.PositionStorage.wasOutputHM2;
import static pedroPathing.newOld.Toggle.toggle_var;

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
import pedroPathing.States.IntakeReverseTruBotOutput;
import pedroPathing.States.IntakeStateExtendedRo2v2;
import pedroPathing.States.IntakeStateRetractedForNoTransfer;
import pedroPathing.States.IntakeStateRetractedRo2;
import pedroPathing.States.IntakeStateWallPURetractionRo2v2;
import pedroPathing.States.IntakeWaitForOutputTruBot;
import pedroPathing.States.OutakeHMandWallPU;
import pedroPathing.States.OuttakeFSM;
import pedroPathing.States.OuttakeSpecimenHang;
import pedroPathing.States.OuttakeStateBasket;
import pedroPathing.States.OuttakeStateSamplePickUp;
import pedroPathing.States.OuttakeStateSpecimen;
import pedroPathing.States.OuttakeStateStandbyDownWithSample;
import pedroPathing.States.OuttakeStateStandbyWithSampleUp;
import pedroPathing.States.OuttakeStateTranfer;
import pedroPathing.newOld.ControlMotor;
import pedroPathing.newOld.PositionStorage;
import pedroPathing.newOld.Toggle;
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

        waitForStart();

        if (isStopRequested()){
            //multiRunnable.stopRunning();
            return;
        }

        while (opModeIsActive()) {
            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            ///gamepad2
            double intakeinput = gamepad2.left_stick_y;
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
















            //PIDs
            double intakeMotorPower = 0;
            intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            double outakeMotorPower;
            outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            outakeMotorPower *= PIDincrement;




            frontRightPowerCat = (pivot - vertical - horizontal);
            backRightPowerCat = (pivot - vertical + horizontal);
            frontLeftPowerCat = (pivot + vertical - horizontal);
            backLeftPowerCat = (pivot + vertical + horizontal);


            //manual slowdown
            double slowyDownyManal = 2.5;
            if(gamepad1.left_bumper){
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
            outakeLeftMotor.setPower(outakeMotorPower);
            intakeSpinMotor.setPower(intakeMotorPickUpPower);


            //Set servo Positions
            intakeRotateServo.setPosition((intakeRotateServoPosition+gravityAdder) / 360);
            outakeArmServo.setPosition(outakeArmServoPosition / 360);
            outakeSampleServo.setPosition(outakeSampleServoPosition / 360);


            updateTelemetry(telemetry);
        }

    }

}