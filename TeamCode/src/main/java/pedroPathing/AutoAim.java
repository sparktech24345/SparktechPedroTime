package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import pedroPathing.constants.FConstantsForPinpoint;
import pedroPathing.constants.LConstantsForPinpoint;

@Config
@TeleOp(name = "Turret aim", group = "LinearOpMode")
public class AutoAim extends LinearOpMode {
    public static double targetX = 0;
    public static double targetY = 25;

    Follower follower;
    MultipleTelemetry multipleTelemetry;
    Servo turretServo;

    private Pose follower_pose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        follower = new Follower(hardwareMap,FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretServo.setPosition(0.5);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();
            follower_pose = follower.getPose();
            follower_pose.setY(-follower_pose.getY());

            double dx = targetX - follower_pose.getX();
            double dy = targetY - follower_pose.getY();

            double robot_angle = Math.toDegrees(follower_pose.getHeading());
            if (robot_angle > 180) robot_angle -= 360;

            double raw_angle = Math.toDegrees(Math.atan2(dy, dx)) - 90 - robot_angle;
            raw_angle = (raw_angle + 540) % 360 - 180;

            double clamp_angle = Math.max(-150, Math.min(150, raw_angle));

            double servo_angle = clamp_angle + 150;
            turretServo.setPosition(servo_angle / 300);

            multipleTelemetry.addData("x", follower_pose.getX());
            multipleTelemetry.addData("y", follower_pose.getY());
            multipleTelemetry.addData("heading", robot_angle);
            multipleTelemetry.addData("servo_given_pos", servo_angle);
            multipleTelemetry.addData("raw_angle", raw_angle);
            multipleTelemetry.update();
        }
    }
}