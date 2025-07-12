package pedroPathing;


import static pedroPathing.ClassWithStates.ColorCompare;
import static pedroPathing.ClassWithStates.currentStateOfSampleInIntake;
import static pedroPathing.ClassWithStates.currentTeam;
import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.ClassWithStates.intakeState;
import static pedroPathing.ClassWithStates.intakeStates;
import static pedroPathing.OrganizedPositionStorage.chassisBackLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisBackRightPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontRightPow;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.intakeSpinMotorPow;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeTargetPosAdder;

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

import kotlin._Assertions;
import pedroPathing.PIDStorageAndUse.ControlMotor;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Drivetrain Brake", group = "Linear OpMode")
public class DrivetrainBrake extends LinearOpMode {

    /// CONFIGS
    ///
    double SPEED_CALC_TIME = 150;
    double Y_BRAKE_COEFF = 0.00015;
    double X_BRAKE_COEFF = 0.00015;

    final float[] hsvValues = new float[3];

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    MultipleTelemetry tel;
    long current_time = System.nanoTime();
    double slowyDownyManal = 0.4;
    double slowyDownyAuto = 0.5;
    double lastTickTime = 0;
    double lastFrontLeftPos = 0;
    double lastFrontRightPos = 0;
    double lastBackLeftPos = 0;
    double lastBackRightPos = 0;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    double ySpeed = 0;
    double xSpeed = 0;
    double turnSpeed = 0;

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

        // Optional: rotation speed
        turnSpeed = (-dFL + dFR - dBL + dBR) / 4.0 / dt;

        lastTickTime = tickTime;

        lastFrontLeftPos = frontLeftPos;
        lastFrontRightPos = frontRightPos;
        lastBackLeftPos = backLeftPos;
        lastBackRightPos = backRightPos;

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


        tel.addData("current team color",currentTeam);
        tel.update();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        lastTickTime = System.currentTimeMillis();
        boolean brakeMode = false;
        while (opModeIsActive()) {
            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            brakeMode = horizontal*horizontal + vertical*vertical <= 0.225;

            if(!brakeMode){
                chassisFrontRightPow = (pivot - vertical - horizontal);
                chassisBackRightPow = (pivot - vertical + horizontal);
                chassisFrontLeftPow = (pivot + vertical - horizontal);
                chassisBackLeftPow = (pivot + vertical + horizontal);
            } else {
                chassisFrontRightPow = (pivot - ySpeed*Y_BRAKE_COEFF - xSpeed*X_BRAKE_COEFF);
                chassisBackRightPow = (pivot - ySpeed*Y_BRAKE_COEFF + xSpeed*X_BRAKE_COEFF);
                chassisFrontLeftPow = (pivot + ySpeed*Y_BRAKE_COEFF - xSpeed*X_BRAKE_COEFF);
                chassisBackLeftPow = (pivot + ySpeed*Y_BRAKE_COEFF + xSpeed*X_BRAKE_COEFF);
            }


            // set motor power
            frontLeftMotor.setPower(chassisFrontLeftPow);
            backLeftMotor.setPower(chassisBackLeftPow);
            frontRightMotor.setPower(chassisFrontRightPow);
            backRightMotor.setPower(chassisBackRightPow);
            intakeMotor.setPower(0);
            outakeRightMotor.setPower(0);
            outakeLeftMotor.setPower(0);
            intakeSpinMotor.setPower(intakeSpinMotorPow);


            //Set servo Positions
            intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
            outakeArmServo.setPosition(outtakePivotServoPos / 328);
            outakeSampleServo.setPosition(outtakeClawServoPos / 360);


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

            //tel.addData("color stuff",currentStateOfSampleInIntake);
            //tel.addData("blue color",colors.blue);
            //tel.addData("red color",colors.red);
            //tel.addData("is color not used", isColorSensorNotInUse);
            //tel.addData("curent team color",currentTeam);
            //tel.addData("Outtake Target Pos Adder",outtakeTargetPosAdder);

            //tel.addData("current time",System.nanoTime());
            //tel.addData("time diference",System.nanoTime() - current_time);
            //current_time = System.nanoTime();

            if (System.currentTimeMillis() >= lastTickTime + SPEED_CALC_TIME)
                calculateSpeeds();

            tel.addData("Y speed", ySpeed);
            tel.addData("X speed", xSpeed);

            tel.addData("fl", lastFrontLeftPos);
            tel.addData("fr", lastFrontRightPos);
            tel.addData("bl", lastBackLeftPos);
            tel.addData("br", lastBackRightPos);

            tel.update();
        }

    }

}