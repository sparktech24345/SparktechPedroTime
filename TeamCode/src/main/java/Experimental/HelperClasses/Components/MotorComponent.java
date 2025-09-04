package Experimental.HelperClasses.Components;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;

import Experimental.HelperClasses.PIDcontroller;

public class MotorComponent extends Component {
    protected HashMap<String, DcMotor> motorMap = new HashMap<>();
    protected DcMotor mainMotor = null;
    protected boolean usePID = false;
    protected PIDcontroller PID = null;

    public MotorComponent() {
        super();
        PID = new PIDcontroller();
    }

    public DcMotor getMotor(String name) {
        return motorMap.get(name);
    }

    public MotorComponent loadState(String s) {
        target = states.get(s);
        return this;
    }

    public MotorComponent setResolution(double res) {
        resolution = res;
        return this;
    }

    public MotorComponent addMotor(String hardwareMapName) {
        DcMotor motor = hardwareMapInstance.get(DcMotor.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
        motorMap.put(hardwareMapName, motor);
        return this;
    }

    public MotorComponent addMotor(String hardwareMapName, String customMapName) {
        DcMotor motor = hardwareMapInstance.get(DcMotor.class, hardwareMapName);
        if (motorMap.isEmpty()) {
            mainMotor = motor;
        }
        motorMap.put(customMapName, motor);
        return this;
    }

    public double getPos() {
        return mainMotor.getCurrentPosition();
    }

    public MotorComponent setBehaviour(DcMotor.ZeroPowerBehavior zeroPower) {
        for (DcMotor motor : motorMap.values()) {
            motor.setZeroPowerBehavior(zeroPower);
        }
        return this;
    }

    public MotorComponent setDirection(String motorName, DcMotorSimple.Direction dir) {
        motorMap.get(motorName).setDirection(dir);
        return this;
    }

    public MotorComponent setPIDconstants(double p, double i, double d) {
        PID.setConstants(p, i, d);
        return this;
    }

    public MotorComponent useWithPIDController(boolean b) {
        usePID = b;
        return this;
    }

    public MotorComponent setRange(double r) {
        range = r;
        return this;
    }

    public DcMotor get(String name) {
        return motorMap.get(name);
    }

    @Override
    public void update() {
        double targetPower = target / resolution;
        if (usePID) {
            targetPower = PID.calculate(target, mainMotor.getCurrentPosition());
        }
        if (range > 0) {
            targetPower = clamp(targetPower, range);
        }
        for (DcMotor motor : motorMap.values()) {
            motor.setPower(targetPower);
        }
    }
}
