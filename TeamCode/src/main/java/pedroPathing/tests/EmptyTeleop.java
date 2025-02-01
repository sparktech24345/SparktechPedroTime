package pedroPathing.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Empty Teleop", group = "Linear OpMode")
@Disabled
public class EmptyTeleop extends LinearOpMode {
    volatile boolean keepRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {

        updateTelemetry(telemetry);
        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("is active",true);
            updateTelemetry(telemetry);
        }

    }
}
