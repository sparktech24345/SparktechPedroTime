package Experimental.HelperClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ExtendedLinearOpMode extends LinearOpMode {
    private ComplexGamepad gamepad = null;
    private MultipleTelemetry tel = null;
    public ExtendedLinearOpMode() {
        super();
        setInstances();
    }
    public void setInstances() {
        GlobalStorage.gamepadInstance = new ComplexGamepad(gamepad1, gamepad2);
        GlobalStorage.hardwareMapInstance = hardwareMap;
        GlobalStorage.telemetryInstance = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.gamepad = GlobalStorage.gamepadInstance;
        this.tel = GlobalStorage.telemetryInstance;
    }
}
