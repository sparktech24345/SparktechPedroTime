package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.ExtendedOpMode;
import Experimental.HelperClasses.GlobalStorage;


@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends ExtendedOpMode {
    private ComplexGamepad gamepad;
    private MultipleTelemetry tel;

    public void init() {
        setInstances();
        gamepad = GlobalStorage.gamepadInstance;
        tel = GlobalStorage.telemetryInstance;

    }

    public void init_loop() {
    }

    public void start() {
    }

    public void loop() {
    }

    public void stop() {
    }
}
