package Experimental;


import static pedroPathing.ClassWithStates.currentTeam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.MainConfig;
import Experimental.Modules.DriveTrain;
import Experimental.Modules.Intake;
import Experimental.Modules.Outtake;
import pedroPathing.ClassWithStates;


@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends OpMode {

    private final MainConfig cfg = new MainConfig();
    private final ComplexGamepad gamepad = new ComplexGamepad();
    private final MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private final DriveTrain drivetrain = new DriveTrain();
    private final Intake intake = new Intake();
    private final Outtake outtake = new Outtake();

    @Override
    public void init() {
        cfg.InitAllStates();
        gamepad.SetGamepads(gamepad1, gamepad2);
        drivetrain.PassGamepad(gamepad);
        drivetrain.PassTelemetry(tel);
        drivetrain.PassHardwareMap(hardwareMap);

        intake.PassGamepad(gamepad);
        intake.PassTelemetry(tel);
        intake.PassHardwareMap(hardwareMap);

        outtake.PassGamepad(gamepad);
        outtake.PassTelemetry(tel);
        outtake.PassHardwareMap(hardwareMap);

        drivetrain.init();
        intake.init();
        outtake.init();
    }

    @Override
    public void init_loop() {
        gamepad.CheckGamepads();
        if ((gamepad.LEFT_BUMPER2.Execute && gamepad.START2.Execute) || (gamepad.LEFT_BUMPER1.Execute && gamepad.START1.Execute))
            currentTeam = ClassWithStates.colorList.blue;
        if ((gamepad.RIGHT_BUMPER2.Execute && gamepad.START2.Execute) || (gamepad.RIGHT_BUMPER1.Execute && gamepad.START1.Execute))
            currentTeam = ClassWithStates.colorList.red;
        tel.addData("COLOR SELECTED",currentTeam);
        tel.update();
    }

    @Override
    public void loop() {
        gamepad.CheckGamepads();
        drivetrain.loop(true);
        intake.loop();
        outtake.loop();
    }
}
