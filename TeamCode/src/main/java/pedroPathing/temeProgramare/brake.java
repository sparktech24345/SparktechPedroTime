package pedroPathing.temeProgramare;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;

enum Direction {Front, LeftFront, RightFront, Left, Right, LeftBack, Back, RightBack, Stopped};
enum Rotation {Left, Right, Stopped};

@TeleOp(name="TeleOP brake", group="Test")

public class brake extends LinearOpMode {

    private static double BrakeMultiplier = -1;
    private static double ProportionalMultiplier = 1;
    private static double rfMultiplier = 1;
    private static double lfMultiplier = 1;
    private static double rbMultiplier = 1;
    private static double lbMultiplier = 1;
    private static double FullSpeedTime = 2500;
    private static double BrakeTimeMs = 1000;
    private static ElapsedTime timer = null;
    private static ElapsedTime BrakeTimer = null;
    private static DcMotor rightFrontDrive = null;
    private static DcMotor leftFrontDrive = null;
    private static DcMotor rightBackDrive = null;
    private static DcMotor leftBackDrive = null;
    private static double speed = 1;
    private static MultipleTelemetry tel;
    private static boolean JoysticksUsedLastTick = false;
    private static boolean IsBraking = false;
    private boolean wasGamepad1upPressed = false;
    private boolean wasGamepad1downPressed = false;
    private boolean wasGamepad1leftPressed = false;
    private boolean wasGamepad1rightPressed = false;
    private Direction DirectionBeforeBrake = Direction.Stopped;
    private Rotation RotationBeforeBrake = Rotation.Stopped;

    @Override
    public void runOpMode() {

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        BrakeTimer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            ModifyVariables();

            double max;

            double lfPower = 0;
            double lbPower = 0;
            double rfPower = 0;
            double rbPower = 0;

            double straight = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double orientation = gamepad1.right_stick_x;

            double rightFrontPower = speed * (straight - lateral - orientation);
            double leftFrontPower = speed * (straight + lateral + orientation);
            double rightBackPower = speed * (straight + lateral - orientation);
            double leftBackPower = speed * (straight - lateral + orientation);

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            CalculateMultipliers();

            if (IsBraking && BrakeTimer.milliseconds() < BrakeTimeMs) {
                BrakeMultiplier = -1;
                leftFrontPower = lfPower;
                leftBackPower = lbPower;
                rightFrontPower = rfPower;
                rightBackPower = rbPower;

                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max != 1 && max != 0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }
                CalculateWheelsMultiplier();
            }
            if (IsBraking && BrakeTimer.milliseconds() >= BrakeTimeMs)
                IsBraking = false;
            if (!IsBraking) {
                BrakeMultiplier = 1;
                ProportionalMultiplier = 1;
                lfPower = leftFrontPower;
                lbPower = leftBackPower;
                rfPower = rightFrontPower;
                rbPower = rightBackPower;
                UpdateDirection();
            }

            leftFrontDrive.setPower(leftFrontPower * ProportionalMultiplier * BrakeMultiplier * ((IsBraking ? 1 : 1/lfMultiplier) * lfMultiplier));
            rightFrontDrive.setPower(rightFrontPower * ProportionalMultiplier * BrakeMultiplier * ((IsBraking ? 1 : 1/rfMultiplier) * rfMultiplier));
            leftBackDrive.setPower(leftBackPower * ProportionalMultiplier * BrakeMultiplier * ((IsBraking ? 1 : 1/lbMultiplier) * lbMultiplier));
            rightBackDrive.setPower(rightBackPower * ProportionalMultiplier * BrakeMultiplier * ((IsBraking ? 1 : 1/rbMultiplier) * rbMultiplier));

            UpdateTelemetry();
        }
    }

    private void CalculateMultipliers() {
        if (!JoysticksUsedLastTick && (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0)) {
            timer.reset();
            JoysticksUsedLastTick = true;
        }

        if (JoysticksUsedLastTick && (gamepad1.left_stick_x == 0 || gamepad1.left_stick_y == 0)) {
            double MoveTime = timer.milliseconds();
            ProportionalMultiplier = MoveTime / FullSpeedTime;
            IsBraking = true;
            BrakeTimer.reset();
            JoysticksUsedLastTick = false;
        }
    }
    private void ModifyVariables() {
        if (gamepad1.dpad_up && !wasGamepad1upPressed) wasGamepad1upPressed = true;
        if (!gamepad1.dpad_up && wasGamepad1upPressed) FullSpeedTime += 25;
        if (!gamepad1.dpad_up) wasGamepad1upPressed = false;

        if (gamepad1.dpad_down && !wasGamepad1downPressed) wasGamepad1downPressed = true;
        if (!gamepad1.dpad_down && wasGamepad1downPressed) FullSpeedTime -= 25;
        if (!gamepad1.dpad_down) wasGamepad1downPressed = false;

        if (gamepad1.dpad_right && !wasGamepad1rightPressed) wasGamepad1rightPressed = true;
        if (!gamepad1.dpad_right && wasGamepad1rightPressed) BrakeTimeMs += 10;
        if (!gamepad1.dpad_right) wasGamepad1rightPressed = false;

        if (gamepad1.dpad_left && !wasGamepad1leftPressed) wasGamepad1leftPressed = true;
        if (!gamepad1.dpad_left && wasGamepad1leftPressed) BrakeTimeMs -= 10;
        if (!gamepad1.dpad_left) wasGamepad1leftPressed = false;
    }

    private void UpdateTelemetry() {
        tel.addData("Runtime", timer.milliseconds());
        tel.addData("BrakeTime", BrakeTimer.milliseconds());
        tel.addLine();
        tel.addData("BrakeTimeMs", BrakeTimeMs);
        tel.addData("FullSpeedTime", FullSpeedTime);
        tel.addLine();
        tel.addData("BrakeMultiplier", BrakeMultiplier);
        tel.addData("ProportionalMultiplier", ProportionalMultiplier);
        tel.addLine();
        tel.addData("Direction", DirectionBeforeBrake);
        tel.addData("Rotation", RotationBeforeBrake);
        tel.update();
    }

    private void UpdateDirection() {
        if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y == 0) {
            DirectionBeforeBrake = Direction.Right;
        }
        else if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y == 0) {
            DirectionBeforeBrake = Direction.Left;
        }
        else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y > 0) {
            DirectionBeforeBrake = Direction.Front;
        }
        else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y < 0) {
            DirectionBeforeBrake = Direction.Back;
        }
        else if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y > 0) {
            DirectionBeforeBrake = Direction.LeftFront;
        }
        else if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y > 0) {
            DirectionBeforeBrake = Direction.RightFront;
        }
        else if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y < 0) {
            DirectionBeforeBrake = Direction.LeftBack;
        }
        else if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_y < 0) {
            DirectionBeforeBrake = Direction.RightBack;
        }
        else DirectionBeforeBrake = Direction.Stopped;

        if (gamepad1.right_stick_x > 0)
            RotationBeforeBrake = Rotation.Right;
        else if (gamepad1.right_stick_x < 0)
            RotationBeforeBrake = Rotation.Left;
        else RotationBeforeBrake = Rotation.Stopped;
    }

    private void CalculateWheelsMultiplier() {
        switch (DirectionBeforeBrake) {
            case Stopped:
                rfMultiplier = 1;
                lfMultiplier = 1;
                rbMultiplier = 1;
                lbMultiplier = 1;
                break;
            case Front:
                rfMultiplier = 0.25;
                lfMultiplier = 0.25;
                rbMultiplier = 1;
                lbMultiplier = 1;
                break;
            case Back:
                rfMultiplier = 1;
                lfMultiplier = 1;
                rbMultiplier = 0.25;
                lbMultiplier = 0.25;
                break;
            case Left:
                rfMultiplier = 1;
                lfMultiplier = 0.25;
                rbMultiplier = 1;
                lbMultiplier = 0.25;
                break;
            case Right:
                rfMultiplier = 0.25;
                lfMultiplier = 1;
                rbMultiplier = 0.25;
                lbMultiplier = 1;
                break;
            default:
                rfMultiplier = 0.8;
                lfMultiplier = 0.8;
                rbMultiplier = 0.8;
                lbMultiplier = 0.8;
                break;
        }
        if (RotationBeforeBrake != Rotation.Stopped) {
            rfMultiplier = 0.7;
            lfMultiplier = 0.7;
            rbMultiplier = 0.7;
            lbMultiplier = 0.7;
        }
    }
}