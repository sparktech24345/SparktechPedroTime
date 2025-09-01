package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Experimental.HelperClasses.Actions.DelayAction;
import Experimental.HelperClasses.Actions.StateAction;
import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;

public class RobotController {
    private ComplexGamepad gamepad;
    private MultipleTelemetry telemetry;
    private HardwareMap hardwareMap;
    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Outtake outtake = new Outtake();
    private StateQueuer queuer = new StateQueuer();
    private double tickMS = 0;
    private NormalizedColorSensor colorSensor;
    private ElapsedTime tickTimer = new ElapsedTime();
    private ColorSet currentColor = ColorSet.Undefined;

    public RobotController() {
        this.gamepad = gamepadInstance;
        this.telemetry = telemetryInstance;
        this.hardwareMap = hardwareMapInstance;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, GlobalStorage.colorSensorName);
        // followerInstance = new ComplexFollower(hardwareMapInstance);
        driveTrainInstance = driveTrain;
        intakeInstance = intake;
        outtakeInstance = outtake;
        queuerInstance = queuer;
    }

    private void setAutoSequence() {
    }

    public void init(OpMode mode) {
        currentOpMode = mode;
        driveTrain.init();
        intake.init();
        outtake.init();
        queuer.addAction(new StateAction(true, RobotState.StartState));
    }

    public void init_loop() {
        gamepad.update();
        if (gamepad.get("LEFT_BUMPER1").IsHeld && gamepad.get("START1").IsHeld)
            currentTeam = ColorSet.Blue;
        else if (gamepad.get("RIGHT_BUMPER1").IsHeld && gamepad.get("START1").IsHeld)
            currentTeam = ColorSet.Red;
        showTelemetry();
    }

    private void runUpdates() {
        currentColor = ColorSet.getColor(colorSensor.getNormalizedColors());
        queuer.update();
        gamepad.update();
        showTelemetry();
    }

    public void loop() {
        tickTimer.reset();
        runUpdates();
        if (currentOpMode == OpMode.TeleOP) HandleControls();
        driveTrain.loop();
        intake.loop();
        outtake.loop();
        tickMS = tickTimer.milliseconds();
    }

    private void HandleControls() {
        if (gamepad.get("X1").ExecuteOnPress && !gamepad.get("X1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(true, RobotState.HighBasket));
        }
        if (gamepad.get("X1").ExecuteOnPress && gamepad.get("X1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(true, RobotState.OpenClaw));
        }
        if (gamepad.get("X1").ExecuteAfterPress && !gamepad.get("X1").IsToggledOnPress) {
            queuer.addAction(new DelayAction(true, 300));
            queuer.addAction(new StateAction(true, RobotState.StandbyState));
        }
        if (gamepad.get("DPAD_UP1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, RobotState.IntakeExtension1));
        }
        if (gamepad.get("DPAD_RIGHT1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, RobotState.IntakeExtension2));
        }
        if (gamepad.get("DPAD_DOWN1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, RobotState.IntakeExtension3));
        }
        if (gamepad.get("DPAD_LEFT1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, RobotState.IntakeExtension4));
        }
        if (gamepad.get("A1").ExecuteOnPress && !gamepad.get("A1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(false, RobotState.SamplePickup));
        }
        if (gamepad.get("A1").IsToggledOnPress) {
            if (gamepad.get("RIGHT_BUMPER1").IsHeld) {
                intakeInstance.setIntakeSpin(-1);
            }
            if (ColorSet.validateSample(currentColor, true)) {
                intakeInstance.setIntakeSpin(0);
                gamepad.get("A1").UnToggle();
                queuer.addAction(new StateAction(true, RobotState.TransferState));
                queuer.addAction(new StateAction(true, RobotState.OpenClaw));
                queuer.addAction(new DelayAction(true, 600));
                queuer.addAction(new StateAction(true, RobotState.CloseClaw));
                queuer.addAction(new DelayAction(true, 200));
                queuer.addAction(new StateAction(true, RobotState.SpecimenHang));
            } else {
                intakeInstance.setIntakeSpin(-1);
            }
        }
        else intakeInstance.setIntakeSpin(0);
    }
    private void showTelemetry() {
        intake.telemetry();
        outtake.telemetry();
//        followerInstance.telemetry();
        telemetry.addData("queue length", queuer.getLen());
        telemetry.addData("State", currentRobotState);
        telemetry.addData("OpMode", currentOpMode);
        telemetry.addData("Team", currentTeam);
        telemetry.addData("Color sensor", currentColor);
        telemetry.addLine();
        telemetry.addData("left stick X", gamepad.get("LEFT_STICK_X1").raw());
        telemetry.addData("left stick Y", gamepad.get("LEFT_STICK_Y1").raw());
        telemetry.addData("right stick X", -gamepad.get("RIGHT_STICK_X1").raw());
        telemetry.addData("tick ms", tickMS);
        telemetry.update();
    }
}
