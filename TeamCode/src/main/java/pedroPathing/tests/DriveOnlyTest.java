package pedroPathing.tests;


import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.newOld.PositionStorage.backLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.backRightPowerCat;
import static pedroPathing.newOld.PositionStorage.frontLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.frontRightPowerCat;
import static pedroPathing.newOld.PositionStorage.intakeActualZero;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.intakeTargetPosAdder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.AutoPIDS.ControlMotor;

@TeleOp(name = "Only Drive Teleop", group = "Linear OpMode")
public class DriveOnlyTest extends LinearOpMode {

    ControlMotor intakeControlMotor = new ControlMotor();
    long current_time;

    @Override
    public void runOpMode() throws InterruptedException {

       DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");

        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);


        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            // executorService.submit(Motors::new);
            intakeTargetPos = 0;


            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            boolean slowdown = gamepad1.left_bumper;

            //calculating nedded power by method 1
            frontRightPowerCat = (pivot - vertical - horizontal);
            backRightPowerCat = (pivot - vertical + horizontal);
            frontLeftPowerCat = (pivot + vertical - horizontal);
            backLeftPowerCat = (pivot + vertical + horizontal);

            //Pivot stuff
            pivot = -pivot;


            if(slowdown) {
                frontRightPowerCat /= 3;
                backRightPowerCat /= 3;
                frontLeftPowerCat /= 3;
                backLeftPowerCat /= 3;
            }

            frontLeftMotor.setPower(frontLeftPowerCat);
            frontRightMotor.setPower(frontRightPowerCat);
            backRightMotor.setPower(backRightPowerCat);
            backLeftMotor.setPower(backLeftPowerCat);

            intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
            outakeArmServo.setPosition(outtakePivotServoPos / 328);
            outakeSampleServo.setPosition(outtakeClawServoPos / 360);

            telemetry.addData("current time",System.nanoTime());

            telemetry.addData("time diference",System.nanoTime() - current_time);
            current_time = System.nanoTime();
            updateTelemetry(telemetry);

        }

    }
}