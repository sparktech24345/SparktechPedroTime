package Experimental.Modules;

import com.qualcomm.robotcore.hardware.*;
import static Experimental.HelperClasses.GlobalStorage.*;
import static Experimental.StatesAndPositions.ColorSet.*;

import Experimental.HelperClasses.GlobalStorage;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;
import pedroPathing.PIDStorageAndUse.NewPidsController;

public class Intake extends BaseModule {

    private NormalizedRGBA currentColorObj;
    private DcMotor IntakeSpin;
    private DcMotor IntakeExtend;
    private Servo IntakeRotation;
    private NewPidsController IntakeControlMotor;
    private NormalizedColorSensor colorSensor;
    private ColorSet currentColor;

    public double actualIntakeExtension = currentIntakeExt.get();

    private double IntakeSpinPower = 0;
    private double IntakeExtendPower = 0;

    public void init() {
        initializeInstances();
        IntakeSpin = hardwareMap.get(DcMotor.class, intakeSpinName);
        IntakeExtend = hardwareMap.get(DcMotor.class, intakeExtendName);
        IntakeRotation = hardwareMap.get(Servo.class, intakePosName);
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, GlobalStorage.colorSensorName);
        IntakeControlMotor = new NewPidsController();

        IntakeSpin.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeRotation.setPosition((currentIntakePos.get() - currentIntakeExt.gravity()) / 228);
    }

    public void loop() {
        actualIntakeExtension = IntakeExtend.getCurrentPosition();
//        currentColorObj = colorSensor.getNormalizedColors();
//        currentColor = getColor(currentColorObj);
        if (gamepad.A1.IsToggled) {
            IntakeSpinPower = 1;
            if (gamepad.LEFT_BUMPER1.IsHeld)
                IntakeSpinPower = -1;
//            if (!sampleIsValid(currentColor, true) && isSample(currentColor))
//                IntakeSpinPower = -1;
        }
        else IntakeSpinPower = 0;
        IntakeExtendPower = IntakeControlMotor.pidControllerIntake(currentIntakeExt.get(), IntakeExtend.getCurrentPosition());
        IntakeExtend.setPower(IntakeExtendPower);
        IntakeSpin.setPower(IntakeSpinPower);
        IntakeRotation.setPosition((currentIntakePos.get() - currentIntakeExt.gravity()) / 228);
    }

    public void showTelemetry() {
        telemetry.addData("IntakeActualPos?", currentIntakePos.get());
//        telemetry.addData("color r", currentColorObj.red);
//        telemetry.addData("color g", currentColorObj.green);
//        telemetry.addData("color b", currentColorObj.blue);
    }
}
