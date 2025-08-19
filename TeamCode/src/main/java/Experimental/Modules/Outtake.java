package Experimental.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.NewPidsController;
import static Experimental.HelperClasses.GlobalStorage.*;

public class Outtake extends BaseModule {

    private DcMotor outtakeExtendLeft;
    private DcMotor outtakeExtendRight;

    private Servo outtakeArmRotation;
    private Servo outtakeClawPos;
    private NewPidsController outtakePIDControl = new NewPidsController();
    public double actualOuttakeExtension = currentOuttakeExt.get();
    private double extensionPow = 0;

    private double outtakeExtendOffset = 0;

    public void init() {
        initializeInstances();
        outtakeExtendRight  = hardwareMap.get(DcMotor.class, outtakeExtendRightName);
        outtakeExtendLeft   = hardwareMap.get(DcMotor.class, outtakeExtendLeftName);
        outtakeArmRotation  = hardwareMap.get(Servo.class, outtakeArmName);
        outtakeClawPos      = hardwareMap.get(Servo.class, outtakeClawName);

        outtakeExtendRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeArmRotation.setPosition(currentOuttakeArmPos.get() / 328);
        outtakeClawPos.setPosition(currentOuttakeClawPos.get() / 360);
    }

    public void loop() {
        actualOuttakeExtension = outtakeExtendLeft.getCurrentPosition();
        extensionPow = outtakePIDControl.pidControllerOuttake((currentOuttakeExt.get() + outtakeExtendOffset), outtakeExtendLeft.getCurrentPosition());
        outtakeExtendRight.setPower(extensionPow);
        outtakeExtendLeft.setPower(extensionPow);
        outtakeArmRotation.setPosition(currentOuttakeArmPos.get() / 328);
        outtakeClawPos.setPosition(currentOuttakeClawPos.get() / 360);
    }

    public void showTelemetry() {
        telemetry.addData("OuttakeArmPos?", currentOuttakeArmPos.get());
        telemetry.addData("OuttakeClawPos?", currentOuttakeClawPos.get());
        telemetry.addData("Extension power", extensionPow);
    }
}
