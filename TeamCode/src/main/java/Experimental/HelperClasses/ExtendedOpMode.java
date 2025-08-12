package Experimental.HelperClasses;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ExtendedOpMode extends OpMode {

    private ComplexGamepad gamepad = null;
    private MultipleTelemetry tel = null;
    public ExtendedOpMode() {
    }
    public void setInstances() {
        gamepad = new ComplexGamepad(gamepad1, gamepad2);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GlobalStorage.gamepadInstance = gamepad;
        GlobalStorage.hardwareMapInstance = hardwareMap;
        GlobalStorage.telemetryInstance = tel;
    }
}
