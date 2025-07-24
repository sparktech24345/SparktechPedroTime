package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Experimental.HelperClasses.ComplexGamepad;


@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends OpMode {
    private ComplexGamepad gamepad;
    private MultipleTelemetry tel;

    public void init() {
        gamepad = new ComplexGamepad(gamepad1, gamepad2);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
