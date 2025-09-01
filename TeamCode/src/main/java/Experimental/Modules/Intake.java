package Experimental.Modules;

import com.qualcomm.robotcore.hardware.*;
import static Experimental.HelperClasses.GlobalStorage.*;
import static Experimental.StatesAndPositions.ColorSet.*;

import Experimental.HelperClasses.GlobalStorage;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;
import pedroPathing.PIDStorageAndUse.NewPidsController;

public class Intake extends BaseModule {

    private boolean IntakeSpinToggle = false;
    private DcMotor IntakeSpin;
    private DcMotor IntakeExtend;
    private Servo IntakeRotation;
    private NewPidsController IntakeControlMotor;
    private double IntakeSpinPower = 0;

    public void setIntakeSpin(double pow) {
        IntakeSpinPower = pow;
    }

    public double getSpin() {
        return IntakeSpinPower;
    }

    public double actualIntakeExtension = currentIntakeExt.get();

    public void init() {
        initializeInstances();
        IntakeSpin = hardwareMap.get(DcMotor.class, intakeSpinName);
        IntakeExtend = hardwareMap.get(DcMotor.class, intakeExtendName);
        IntakeRotation = hardwareMap.get(Servo.class, intakePosName);

//        IntakeSpin.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeRotation.setPosition((currentIntakePos.get() - currentIntakeExt.gravity()) / 228);
    }

    public void loop() {
        actualIntakeExtension = IntakeExtend.getCurrentPosition();
        double IntakeExtendPower = NewPidsController.pidControllerIntake(currentIntakeExt.get(), IntakeExtend.getCurrentPosition());
        IntakeExtend.setPower(IntakeExtendPower);
        IntakeSpin.setPower(IntakeSpinPower);
        IntakeRotation.setPosition((currentIntakePos.get() - currentIntakeExt.gravity()) / 228);
    }

    public void telemetry() {
    }
}
