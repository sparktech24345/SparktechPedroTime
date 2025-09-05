package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import android.util.Pair;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import Experimental.HelperClasses.Actions.Action;
import Experimental.HelperClasses.Components.Component;
import Experimental.StatesAndPositions.ColorSet;

public class RobotController {
    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();
    private ColorSet currentColor = ColorSet.Undefined;
    private HashMap<String, RobotState> states = new HashMap<>();
    private HashMap<String, Component> components = new HashMap<>();
    private boolean useDefaultMovement = false;
    private DriveTrain movement = null;
    private Runnable controlBehaviour = () -> {};
    private Runnable telemetry = () -> {};
    private ArrayList<Pair<BooleanSupplier, Runnable>> callbacks = new ArrayList<>();
    private HashMap<String, Pose> autoPositions = new HashMap<>();

    public RobotController() {
        followerInstance = new ComplexFollower(hardwareMapInstance);
        queuerInstance = new StateQueuer();
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

    public RobotController addRobotState(String stateName, RobotState state) {
        states.put(stateName, state);
        return this;
    }

    public RobotController addToQueue(Action action) {
        queuerInstance.addAction(action);
        return this;
    }

    public RobotController addToQueue(Action... actions) {
        queuerInstance.addAction(actions);
        return this;
    }

    public Button getControllerKey(String name) {
        return gamepadInstance.get(name);
    }

    public Pose getAutoPose(String name) {
        return autoPositions.get(name);
    }

    public RobotController addCallback(BooleanSupplier exec, Runnable run) {
        callbacks.add(new Pair<>(exec, run));
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

    public RobotController setControls(Runnable controls) {
        this.controlBehaviour = controls;
        return this;
    }

    public RobotController setTelemetry(Runnable tele) {
        telemetry = tele;
        return this;
    }

    public RobotController addAutoPosition(String name, double x, double y, double heading) {
        autoPositions.put(name, new Pose(x, y, Math.toRadians(heading)));
        return this;
    }

    public RobotController addAutoPosition(String name, Pose position) {
        autoPositions.put(name, position);
        return this;
    }

    public RobotController setAutoStartingPos(String name) {
        followerInstance.getInstance().setStartingPose(autoPositions.get(name));
        return this;
    }

    public RobotController loadRobotState(String robotState) {
        RobotState state = states.get(robotState);
        HashMap<String, String> positions = state.getPositions();
        for (Map.Entry<String, String> entry : positions.entrySet()) {
            components.get(entry.getKey()).loadState(entry.getValue());
        }
        return this;
    }

    public RobotController init(OpModes mode) {
        currentOpModes = mode;
        return this;
    }

    public RobotController init_loop() {
        gamepadInstance.update();
        for (Component c : components.values()) {
            if (c.moveDuringInit()) {
                c.update();
            }
        }
        showTelemetry();
        return this;
    }

    private void runUpdates() {
        gamepadInstance.update();
        queuerInstance.update();
        for (Component c : components.values()) {
            c.update();
        }
        if (movement != null) movement.loop();
        showTelemetry();
    }

    public RobotController loop() {
        tickTimer.reset();
        runUpdates();
        if (currentOpModes == OpModes.TeleOP) {
            controlBehaviour.run();
            for (Pair<BooleanSupplier, Runnable> pair : callbacks) {
                if (pair.first.getAsBoolean()) {
                    pair.second.run();
                }
            }
        }
        tickMS = tickTimer.milliseconds();
        return this;
    }

    public double getExecMS() { return tickMS; };

    private void showTelemetry() {
        if (followerInstance != null) followerInstance.telemetry();
        if (movement != null) movement.telemetry();
        telemetry.run();
        telemetryInstance.update();
    }
}
