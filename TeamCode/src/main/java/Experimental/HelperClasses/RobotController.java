package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.HelperClasses.Actions.StateAction;
import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
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

    public RobotController() {
        followerInstance = new ComplexFollower(new Follower(hardwareMap, F_Constants, L_Constants));
        this.gamepad = gamepadInstance;
        this.telemetry = telemetryInstance;
        this.hardwareMap = hardwareMapInstance;
        this.follower = followerInstance;
        driveTrainInstance = driveTrain;
        intakeInstance = intake;
        outtakeInstance = outtake;
    }

    public void init(OpMode mode) {
        currentOpMode = mode;
        driveTrain.init();
        intake.init();
        outtake.init();
        queuer.addAction(new StateAction(true, RobotState.StartState));
    }

    public void init_loop() {
        runUpdates();
        if (gamepad.LEFT_BUMPER1.IsHeld && gamepad.START1.IsHeld)
            currentTeam = ColorSet.Blue;
        if (gamepad.RIGHT_BUMPER1.IsHeld && gamepad.START1.IsHeld)
            currentTeam = ColorSet.Red;
    }

    private void runUpdates() {
        follower.update();
        queuer.update();
        gamepad.CheckGamepads();
        showTelemetry();
    }

    public void loop() {
        runUpdates();
        HandleControls();
        driveTrain.loop();
        intake.loop();
        outtake.loop();
    }

    private void HandleControls() {
        if (gamepad.X1.Execute) {
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
        follower.telemetry(telemetry);
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
        telemetry.update();
    }
}
