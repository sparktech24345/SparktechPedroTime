package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import Experimental.StatesAndPositions.ColorSet;

public class RobotController {
    private ComplexGamepad gamepad;
    private MultipleTelemetry telemetry;
    private HardwareMap hardwareMap;
    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Outtake outtake = new Outtake();
    private StateQueuer states = new StateQueuer();

    public RobotController(ComplexGamepad gamepad, MultipleTelemetry telemetry, HardwareMap hardwareMap) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init(OpMode mode) {
        driveTrain.PassElements(gamepad, telemetry, hardwareMap);
        intake.PassElements(gamepad, telemetry, hardwareMap);
        outtake.PassElements(gamepad, telemetry, hardwareMap);
        states.setState(RobotState.StartState);
        states.loadState();
        driveTrain.init();
        intake.init();
        outtake.init();
    }

    public void init_loop() {
        gamepad.CheckGamepads();
        if (gamepad.LEFT_BUMPER2.IsHeld && gamepad.START2.IsHeld)
            currentTeam = ColorSet.Blue;
        if (gamepad.RIGHT_BUMPER2.IsHeld && gamepad.START2.IsHeld)
            currentTeam = ColorSet.Red;
        showTelemetry();
    }

    public void loop() {
        gamepad.CheckGamepads();
        HandleControls();
        driveTrain.loop();
        intake.loop();
        outtake.loop();
        showTelemetry();
    }

    private void HandleControls() {
        if (gamepad.X1.Execute)
            states.setState(RobotState.StandbyState);
        if (gamepad.DRIGHT1.Execute)
            states.setState(RobotState.SampleTransferReadyState);
        states.loadState();
    }

    private void showTelemetry() {
        intake.showTelemetry();
        telemetry.addData("State", currentRobotState);
        telemetry.addData("intakeExt", currentIntakeExt);
        telemetry.addData("intakePos", currentIntakePos);
        telemetry.addData("outtakeExt", currentOuttakeExt);
        telemetry.addData("outtakeArm", currentOuttakeArmPos);
        telemetry.addData("outtakeClaw", currentOuttakeClawPos);
        telemetry.addData("OpMode", currentOpMode);
        telemetry.addData("Team", currentTeam);
        telemetry.update();
    }
}
