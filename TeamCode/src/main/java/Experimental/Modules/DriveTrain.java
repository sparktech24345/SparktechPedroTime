package Experimental.Modules;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Experimental.HelperClasses.OpMode;

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
@com.acmerobotics.dashboard.config.Config
public class DriveTrain extends BaseModule {

    private DcMotor RFDrive;
    private DcMotor LFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;
    private boolean directionFlip = false;

    public void init() {
        initializeInstances();
        if (currentOpMode == OpMode.TeleOP) {
            RFDrive = hardwareMap.get(DcMotor.class, frontRightName);
            LFDrive = hardwareMap.get(DcMotor.class, frontLeftName);
            RBDrive = hardwareMap.get(DcMotor.class, backRightName);
            LBDrive = hardwareMap.get(DcMotor.class, backLeftName);

            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        double vertical = -gamepad.get("LEFT_STICK_Y1").raw();  // Note: pushing stick forward gives negative value
        double horizontal =  gamepad.get("LEFT_STICK_X1").raw();
        double pivot =  gamepad.get("RIGHT_STICK_X1").raw();

        double FrontRightPow = vertical + horizontal - pivot;
        double BackRightPow = vertical - horizontal - pivot;
        double FrontLeftPow = vertical - horizontal + pivot;
        double BackLeftPow = vertical + horizontal + pivot;

        RFDrive.setPower(FrontRightPow);
        LFDrive.setPower(FrontLeftPow);
        RBDrive.setPower(BackRightPow);
        LBDrive.setPower(BackLeftPow);
    }

    private void auto_loop() {

    }

    public void stop() {}
}
