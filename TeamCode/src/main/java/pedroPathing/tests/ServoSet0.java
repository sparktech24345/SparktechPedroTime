package pedroPathing.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoSet0", group = "Linear OpMode")

public class ServoSet0 extends LinearOpMode {
    private int pos=0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo tester = hardwareMap.get(Servo.class, "outakeArmServo");
        pos=0;
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.a) pos++;
            if(gamepad1.b) pos--;

            tester.setPosition(pos);
            telemetry.addData("position", tester.getPosition());
            updateTelemetry(telemetry);
        }

    }
}