package pedroPathing.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Empty Teleop", group = "Linear OpMode")
public class EmptyTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("is in init",true);
        updateTelemetry(telemetry);

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("is active",true);
            updateTelemetry(telemetry);
        }

    }
}