package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

import java.util.HashMap;

import Experimental.HelperClasses.Actions.Action;
import Experimental.HelperClasses.Actions.DelayAction;
import Experimental.HelperClasses.Actions.StateAction;
import Experimental.HelperClasses.Components.Component;
import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;

public class RobotController {
    private ComplexGamepad gamepad;
    private MultipleTelemetry telemetry;
    private HardwareMap hardwareMap;
    private StateQueuer queuer = new StateQueuer();
    private double tickMS = 0;
    private NormalizedColorSensor colorSensor;
    private ElapsedTime tickTimer = new ElapsedTime();
    private ColorSet currentColor = ColorSet.Undefined;
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private boolean useDefaultMovement = false;
    private DriveTrain movement = null;

    public RobotController() {
        this.gamepad = gamepadInstance;
        this.telemetry = telemetryInstance;
        this.hardwareMap = hardwareMapInstance;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, GlobalStorage.colorSensorName);
        // followerInstance = new ComplexFollower(hardwareMapInstance);
        queuerInstance = queuer;
        robotControllerInstance = this;
    }

    public RobotController makeComponent(String name, Component component) {
        components.put(name, component);
        return this;
    }

    public RobotController setState(String robotState) {
        RobotState state = states.get(robotState);
        HashMap<String, String> poses = state.getPositions();
        Component comp = null;
        for (String s : poses.keySet()) {
            comp = components.get(s);
            comp.loadState(poses.get(s));
        }
        return this;
    }

    public RobotController addState(String stateName, RobotState state) {
        states.put(stateName, state);
        return this;
    }

    public RobotController addToQueue(Action action) {
        queuer.addAction(action);
        return this;
    }

    public <T extends Component> T getComponent(String componentName) {
        return (T) components.get(componentName);
    }

    public RobotController UseDefaultMovement(String LeftFront, String RightFront, String LeftBack, String RightBack) {
        movement = new DriveTrain(frontLeftName, frontRightName, backLeftName, backLeftName);
        return this;
    }

    public RobotController UseDefaultMovement() {
        movement = new DriveTrain();
        return this;
    }

    public void init(OpMode mode) {
        currentOpMode = mode;
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
        for (Component c : components.values()) {
            c.update();
        }
        if (movement != null) movement.loop();
        queuer.update();
        gamepad.update();
        showTelemetry();
    }

    public void loop() {
        tickTimer.reset();
        runUpdates();
        if (currentOpMode == OpMode.TeleOP) HandleControls();
        tickMS = tickTimer.milliseconds();
    }

    private void HandleControls() {
        if (gamepad.get("X1").ExecuteOnPress && !gamepad.get("X1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(true, "OUTTAKE_EXTENSION", "MAX_HIGH_BASKET"));
            queuer.addAction(new StateAction(true, "OUTTAKE_ARM", "HIGH_RUNG"));
        }
        if (gamepad.get("X1").ExecuteOnPress && gamepad.get("X1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(true, "OUTTAKE_EXTENSION", "MAX_LOW_BASKET"));
            queuer.addAction(new StateAction(true, "OUTTAKE_ARM", "BASKET_SCORE"));
        }
        if (gamepad.get("X1").ExecuteAfterPress && !gamepad.get("X1").IsToggledOnPress) {
            queuer.addAction(new DelayAction(true, 300));
            queuer.addAction(new StateAction(true, "OUTTAKE_ARM", "HIGH_RUNG"));
            queuer.addAction(new StateAction(true, "OUTTAKE_EXTENSION", "ABSOLUTE_ZERO"));
        }
        if (gamepad.get("DPAD_UP1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED1"));
        }
        if (gamepad.get("DPAD_RIGHT1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED2"));
        }
        if (gamepad.get("DPAD_DOWN1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED3"));
        }
        if (gamepad.get("DPAD_LEFT1").ExecuteOnPress) {
            queuer.addAction(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED4"));
        }
        if (gamepad.get("A1").ExecuteOnPress && !gamepad.get("A1").IsToggledAfterPress) {
//            queuer.addAction(new StateAction(false, RobotState.SamplePickup));
        }
        if (gamepad.get("A1").IsToggledOnPress) {
            if (gamepad.get("RIGHT_BUMPER1").IsHeld) {
//                intakeInstance.setIntakeSpin(-1);
            }
            if (ColorSet.validateSample(currentColor, true)) {
//                intakeInstance.setIntakeSpin(0);
                gamepad.get("A1").UnToggle();
//                queuer.addAction(new StateAction(true, RobotState.TransferState));
//                queuer.addAction(new StateAction(true, RobotState.OpenClaw));
                queuer.addAction(new DelayAction(true, 600));
//                queuer.addAction(new StateAction(true, RobotState.CloseClaw));
                queuer.addAction(new DelayAction(true, 200));
//                queuer.addAction(new StateAction(true, RobotState.SpecimenHang));
            } else {
//                intakeInstance.setIntakeSpin(-1);
            }
        }
//        else intakeInstance.setIntakeSpin(0);
    }

    private void showTelemetry() {
//        followerInstance.telemetry();
        telemetry.addData("queue length", queuer.getLen());
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
