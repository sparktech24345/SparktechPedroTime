package Experimental.Modules;

import com.qualcomm.robotcore.hardware.*;
import static Experimental.HelperClasses.GlobalStorage.*;
import static Experimental.StatesAndPositions.ColorSet.*;

import Experimental.HelperClasses.GlobalStorage;
import Experimental.StatesAndPositions.ColorSet;
import pedroPathing.PIDStorageAndUse.ControlMotor;

public class Intake extends BaseModule {

    private DcMotor IntakeSpin;
    private DcMotor IntakeExtend;
    private Servo IntakeRotation;
    private ControlMotor IntakeControlMotor;
    private NormalizedColorSensor colorSensor;

    private ColorSet currentColor;

    private double IntakeSpinPower = 0;
    private double IntakeExtendPower = 0;

    public void init() {
        IntakeSpin = hardwareMap.get(DcMotor.class, intakeSpinName);
        IntakeExtend = hardwareMap.get(DcMotor.class, intakeExtendName);
        IntakeRotation = hardwareMap.get(Servo.class, intakePosName);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, GlobalStorage.colorSensorName);
        IntakeControlMotor = new ControlMotor();

        IntakeExtend.setDirection(DcMotor.Direction.REVERSE);

        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeRotation.setPosition(currentIntakePos.get() / 228);
        currentColor = getColor(colorSensor.getNormalizedColors());
    }

    public void loop() {
        currentColor = getColor(colorSensor.getNormalizedColors());
        if (gamepad.A1.IsToggled) {
            IntakeSpinPower = 1;
            if (gamepad.LEFT_BUMPER1.IsHeld)
                IntakeSpinPower = -1;
            if (!sampleIsValid(currentColor, true) && isSample(currentColor))
                IntakeSpinPower = -1;
        }
        else IntakeSpinPower = 0;
        IntakeExtendPower = IntakeControlMotor.PIDControl(currentIntakeExt.get(), IntakeExtend.getCurrentPosition());
        IntakeExtend.setPower(IntakeExtendPower);
        IntakeSpin.setPower(IntakeSpinPower);
        IntakeRotation.setPosition(currentIntakePos.get());
    }
}
