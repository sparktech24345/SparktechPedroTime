package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private ElapsedTime tickTimer = new ElapsedTime();

    public RobotController() {
        this.gamepad = gamepadInstance;
        this.telemetry = telemetryInstance;
        this.hardwareMap = hardwareMapInstance;
        followerInstance = new ComplexFollower(hardwareMapInstance);
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
        queuer.update();
        gamepad.update();
        showTelemetry();
        //queuer.addAction(new MoveAction(true, new Pose(-5, 0), true));
        //queuer.addAction(new MoveAction(true, new Pose(5, 0), true));
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
            queuer.addAction(new StateAction(true, RobotState.HighBasketScoreReadyState));
        }
        if (gamepad.get("X1").ExecuteOnPress && gamepad.get("X1").IsToggledAfterPress) {
            queuer.addAction(new StateAction(true, RobotState.HighBasketScoreDoneState));
        }
        if (gamepad.get("X1").ExecuteAfterPress && !gamepad.get("X1").IsToggledOnPress) {
            queuer.addAction(new DelayAction(true, 300));
            queuer.addAction(new StateAction(true, RobotState.StandbyState));
        }
        if (gamepad.get("DPAD_UP1").ExecuteOnPress) {
            currentIntakeExt = IntakeExtension.Extended1;
        }
        if (gamepad.get("DPAD_RIGHT1").ExecuteOnPress) {
            currentIntakeExt = IntakeExtension.Extended2;
        }
        if (gamepad.get("DPAD_DOWN1").ExecuteOnPress) {
            currentIntakeExt = IntakeExtension.Extended3;
        }
        if (gamepad.get("DPAD_LEFT1").ExecuteOnPress) {
            currentIntakeExt = IntakeExtension.Extended4;
        }
    }
    private void showTelemetry() {
        intake.telemetry();
        outtake.telemetry();
        followerInstance.telemetry();
        telemetry.addData("Global queue is empty?", queuer.isEmpty());
        telemetry.addData("State", currentRobotState);
        telemetry.addData("intakeExt", currentIntakeExt);
        telemetry.addData("intakePos", currentIntakePos);
        telemetry.addData("outtakeExt", currentOuttakeExt);
        telemetry.addData("outtakeArm", currentOuttakeArmPos);
        telemetry.addData("outtakeClaw", currentOuttakeClawPos);
        telemetry.addData("OpMode", currentOpMode);
        if (currentTeam == ColorSet.Black)
            telemetry.addData("Team", "Not set");
        else
            telemetry.addData("Team", currentTeam);
        telemetry.addLine();
        telemetry.addData("left stick X", gamepad.get("LEFT_STICK_X1").raw());
        telemetry.addData("left stick Y", gamepad.get("LEFT_STICK_Y1").raw());
        telemetry.addData("right stick X", -gamepad.get("RIGHT_STICK_X1").raw());
        telemetry.addData("tick ms", tickMS);
        telemetry.update();
    }
}
