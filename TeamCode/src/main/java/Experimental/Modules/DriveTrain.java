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
    private String frontLeft = frontLeftName;
    private String frontRight = frontRightName;
    private String backLeft = backLeftName;
    private String backRight = backRightName;

    public DriveTrain() {
        init();
    }

    public DriveTrain(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        frontLeft = LeftFront;
        frontRight = RightFront;
        backLeft = LeftBack;
        backRight = RightBack;
        init();
    }

    public void init() {
        initializeInstances();
        if (currentOpMode == OpMode.TeleOP) {
            RFDrive = hardwareMap.get(DcMotor.class, frontRight);
            LFDrive = hardwareMap.get(DcMotor.class, frontLeft);
            RBDrive = hardwareMap.get(DcMotor.class, backRight);
            LBDrive = hardwareMap.get(DcMotor.class, backLeft);

            RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void start() {}

    public void loop() {

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

    public void stop() {}
}
