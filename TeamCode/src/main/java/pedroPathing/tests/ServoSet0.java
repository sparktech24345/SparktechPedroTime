package pedroPathing.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoSet0", group = "Linear OpMode")
@Disabled
public class ServoSet0 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Servo tester = hardwareMap.get(Servo.class, "outakeArmServo");

        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {
            tester.setPosition(0);
            telemetry.addData("position", tester.getPosition());
            updateTelemetry(telemetry);
        }

    }
}