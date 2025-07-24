package Experimental.HelperClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.nio.channels.MulticastChannel;

import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;

public class RobotController {
    private ComplexGamepad gamepad;
    private MultipleTelemetry telemetry;
    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Outtake outtake = new Outtake();

    public RobotController(ComplexGamepad gamepad, MultipleTelemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void init(OpMode mode) {

    }
}
