package Experimental.Modules;

import static Experimental.HelperClasses.GlobalStorage.*;
import static Experimental.StatesAndPositions.AutoOfSpecStatesAndPos.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

import Experimental.HelperClasses.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/*
* 
*  !! IMPORTANT !!
* 
*  THE DRIVETRAIN CLASS USES THE FOLLOWING ABBREVIATIONS:
*  RF - Right Front
*  LF - Left Front
*  RB - Right Back
*  LB - Left Back
* 
* */
public class DriveTrain extends BaseModule {

    private DcMotor RFDrive;
    private DcMotor LFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;
//    private Follower follower;
    private boolean directionFlip = false;

    public void init() {
        if (currentOpMode == OpMode.TeleOP) {
        RFDrive = hardwareMap.get(DcMotor.class, frontRightName);
        LFDrive = hardwareMap.get(DcMotor.class, frontLeftName);
        RBDrive = hardwareMap.get(DcMotor.class, backRightName);
        LBDrive = hardwareMap.get(DcMotor.class, backLeftName);

        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
//            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//            follower.setStartingPose(startPose);
//            follower.update();
        }
    }

    public void start() {}

    public void loop() {
        if (currentOpMode == OpMode.TeleOP) {
            teleop_loop();
        }
        else auto_loop();
    }

    private void teleop_loop() {
        double directionMulti = 1 + eval(directionFlip) * -2;

        if (gamepad.RIGHT_TRIGGER1.IsHeld && gamepad.LEFT_TRIGGER1.IsHeld &&
            (gamepad.RIGHT_TRIGGER1.Execute  || gamepad.LEFT_TRIGGER1.Execute))
            directionFlip = !directionFlip;


        double  vertical    =  gamepad.gamepad1.left_stick_y * directionMulti,
                horizontal  =  gamepad.gamepad1.left_stick_x * directionMulti,
                pivot       = -gamepad.gamepad1.right_stick_x;

        double backLeftPow      = pivot + vertical + horizontal;
        double frontLeftPow     = pivot + vertical - horizontal;
        double backRightPow     = pivot - vertical + horizontal;
        double frontRightPow    = pivot - vertical - horizontal;

        double max = Math.max(backLeftPow, Math.max(backRightPow, Math.max(frontLeftPow, frontRightPow)));
        if (Math.abs(max) > 1) {
            backRightPow  /= max;
            backLeftPow   /= max;
            frontLeftPow  /= max;
            frontRightPow /= max;
        }

        RFDrive.setPower(frontRightPow);
        LFDrive.setPower(frontLeftPow);
        RBDrive.setPower(backRightPow);
        LBDrive.setPower(backLeftPow);
    }

    private void auto_loop() {

    }

    public void stop() {}
}
