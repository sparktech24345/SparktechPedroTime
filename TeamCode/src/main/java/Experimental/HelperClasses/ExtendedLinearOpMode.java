package Experimental.HelperClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ExtendedLinearOpMode extends LinearOpMode {
    private ComplexGamepad gamepad = null;
    private MultipleTelemetry tel = null;
    public ExtendedLinearOpMode() {
        gamepad = new ComplexGamepad(gamepad1, gamepad2);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void setInstances() {
        GlobalStorage.gamepadInstance = gamepad;
        GlobalStorage.hardwareMapInstance = hardwareMap;
        GlobalStorage.telemetryInstance = tel;
    }
}
