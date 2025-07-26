package Experimental.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import static Experimental.HelperClasses.GlobalStorage.*;

public class Outtake extends BaseModule {

    private DcMotor outtakeExtendLeft;
    private DcMotor outtakeExtendRight;

    private Servo outtakeArmRotation;
    private Servo outtakeClawPos;
    private ControlMotor outtakeController = new ControlMotor();

    private double outtakeExtendOffset = 0;

    public void init() {
        outtakeExtendRight  = hardwareMap.get(DcMotor.class, outtakeExtendRightName);
        outtakeExtendLeft   = hardwareMap.get(DcMotor.class, outtakeExtendLeftName);
        outtakeArmRotation  = hardwareMap.get(Servo.class, outtakeArmName);
        outtakeClawPos      = hardwareMap.get(Servo.class, outtakeClawName);

        outtakeExtendRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeArmRotation.setPosition(currentOuttakeArmPos.get() / 328);
        outtakeClawPos.setPosition(currentOuttakeClawPos.get() / 360);
    }

    public void loop() {
        double extensionPow = outtakeController.PIDControlUppy((currentOuttakeExt.get() + outtakeExtendOffset), outtakeExtendLeft.getCurrentPosition());
        outtakeExtendRight.setPower(extensionPow);
        outtakeExtendLeft.setPower(extensionPow);
        outtakeClawPos.setPosition(currentOuttakeClawPos.get() / 328);
        outtakeArmRotation.setPosition(currentOuttakeArmPos.get() / 360);
    }
}
