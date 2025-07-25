package Experimental.Modules;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.HelperClasses.ComplexGamepad;

public abstract class BaseModule {
    protected ComplexGamepad gamepad;
    protected MultipleTelemetry telemetry;
    protected HardwareMap hardwareMap;
    public void PassElements(ComplexGamepad gamepad, MultipleTelemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void PassGamepad(ComplexGamepad gamepad) {
        this.gamepad = gamepad;
    }
    public void PassTelemetry(MultipleTelemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void PassHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
