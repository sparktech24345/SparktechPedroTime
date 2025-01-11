package pedroPathing.tests;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU TEST", group = "Linear OpMode")
@Disabled
public class IMUTEST extends LinearOpMode {
    IMU imu = null;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo tester = hardwareMap.get(Servo.class, "outakeArmServo");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {

            float angle = (float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU angular velocity", imu.getRobotYawPitchRollAngles());
            telemetry.addData("angle", angle);
            //telemetry.addData("getRobotOrientationAsQuaternion", imu.getRobotOrientationAsQuaternion());
            //telemetry.addData("getRobotYawPitchRollAngles", imu.getRobotYawPitchRollAngles());
            updateTelemetry(telemetry);
        }

    }
}