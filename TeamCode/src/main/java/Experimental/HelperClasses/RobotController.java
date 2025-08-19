package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import Experimental.HelperClasses.Actions.DelayAction;
import Experimental.HelperClasses.Actions.MoveAction;
import Experimental.HelperClasses.Actions.StateAction;
import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import Experimental.StatesAndPositions.AutoOfSpecStatesAndPos;
import Experimental.StatesAndPositions.ColorSet;
import Experimental.StatesAndPositions.IntakeExtension;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class RobotController {
    private ComplexGamepad gamepad;
    private MultipleTelemetry telemetry;
    private HardwareMap hardwareMap;
    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Outtake outtake = new Outtake();
    private ComplexFollower follower;
    private StateQueuer queuer = new StateQueuer();
    private boolean runAuto = false;

    private double tickMS = 0;
    private ElapsedTime tickTimer = new ElapsedTime();

    public RobotController() {
        this.gamepad = gamepadInstance;
        this.telemetry = telemetryInstance;
        this.hardwareMap = hardwareMapInstance;
        followerInstance = new ComplexFollower(hardwareMapInstance);
        this.follower = followerInstance;
        driveTrainInstance = driveTrain;
        intakeInstance = intake;
        outtakeInstance = outtake;
    }

    private void setAutoSequence() {
        queuer.addAction(new MoveAction(true, new Pose(-10, 0), true));
        queuer.addAction(new MoveAction(true, new Pose(10, 0), true));
    }

    public void init(OpMode mode) {
        currentOpMode = mode;
        if (currentOpMode == OpMode.Autonomous && !runAuto) {
            runAuto = true;
        }
        driveTrain.init();
        intake.init();
        outtake.init();
        queuer.addAction(new StateAction(true, RobotState.StartState));
    }

    public void init_loop() {
        gamepad.CheckGamepads();
        if (gamepad.LEFT_BUMPER1.IsHeld && gamepad.START1.IsHeld)
            currentTeam = ColorSet.Blue;
        if (gamepad.RIGHT_BUMPER1.IsHeld && gamepad.START1.IsHeld)
            currentTeam = ColorSet.Red;
        showTelemetry();
    }

    private void runUpdates() {
        queuer.update();
        if (queuer.isEmpty() && currentOpMode == OpMode.Autonomous) setAutoSequence();
        follower.update();
        gamepad.CheckGamepads();
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
        if (gamepad.X1.Execute) {
            queuer.addAction(new StateAction(true, RobotState.SpecimenHangReadyState));
            queuer.addAction(new DelayAction(true, 400));
            queuer.addAction(new StateAction(true, RobotState.StartState));
        }
        if (gamepad.DRIGHT1.Execute) {
            queuer.addAction(new StateAction(true, RobotState.HighBasketScoreReadyState));
        }
        if (gamepad.DDOWN1.Execute)
            currentIntakeExt = IntakeExtension.Extended2;
        if (gamepad.DUP1.Execute)
            currentIntakeExt = IntakeExtension.Retracted;
    }
    private void showTelemetry() {
        intake.showTelemetry();
        outtake.showTelemetry();
        follower.telemetry();
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
        telemetry.addData("left stick X", gamepad.gamepad1.left_stick_x);
        telemetry.addData("left stick Y", gamepad.gamepad1.left_stick_y);
        telemetry.addData("right stick X", -gamepad.gamepad1.right_stick_x);
        telemetry.addData("tick ms", tickMS);
        telemetry.update();
    }
}
