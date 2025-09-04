package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
public class DriveTrain {

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
        if (currentOpModes == OpModes.TeleOP) {
            RFDrive = hardwareMapInstance.get(DcMotor.class, frontRight);
            LFDrive = hardwareMapInstance.get(DcMotor.class, frontLeft);
            RBDrive = hardwareMapInstance.get(DcMotor.class, backRight);
            LBDrive = hardwareMapInstance.get(DcMotor.class, backLeft);

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

        double vertical = -gamepadInstance.get("LEFT_STICK_Y1").raw();  // Note: pushing stick forward gives negative value
        double horizontal =  gamepadInstance.get("LEFT_STICK_X1").raw();
        double pivot =  gamepadInstance.get("RIGHT_STICK_X1").raw();

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

    public void telemetry() {
    }
}
