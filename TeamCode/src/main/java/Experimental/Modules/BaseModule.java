package Experimental.Modules;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.GlobalStorage;

public abstract class BaseModule {
    protected ComplexGamepad gamepad;
    protected MultipleTelemetry telemetry;
    protected HardwareMap hardwareMap;
    public void initializeInstances() {
        this.gamepad = GlobalStorage.gamepadInstance;
        this.telemetry = GlobalStorage.telemetryInstance;
        this.hardwareMap = GlobalStorage.hardwareMapInstance;
    }
}
