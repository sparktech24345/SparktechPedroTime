package pedroPathing;

import static pedroPathing.OrganizedPositionStorage.chassisBackLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisBackRightPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontLeftPow;
import static pedroPathing.OrganizedPositionStorage.chassisFrontRightPow;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.ControlMotor;

@TeleOp(name = "test", group = "Linear OpMode")
public class test extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    public void runOpMode() throws InterruptedException {

        OrganizedPositionStorage.resetStuff();

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        backLeftMotor = hardwareMap.dcMotor.get("backleft");
        frontRightMotor = hardwareMap.dcMotor.get("frontright");
        backRightMotor = hardwareMap.dcMotor.get("backright");

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()){
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;

            chassisFrontRightPow = (pivot - vertical - horizontal);
            chassisBackRightPow = (pivot - vertical + horizontal);
            chassisFrontLeftPow = (pivot + vertical - horizontal);
            chassisBackLeftPow = (pivot + vertical + horizontal);

            frontLeftMotor.setPower(chassisFrontLeftPow);
            backLeftMotor.setPower(chassisBackLeftPow);
            frontRightMotor.setPower(chassisFrontRightPow);
            backRightMotor.setPower(chassisBackRightPow);
        }
    }
}
