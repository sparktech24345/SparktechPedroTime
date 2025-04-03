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
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        pos=0;
        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) pos++;
            if(gamepad1.b) pos--;
            outakeArmServo.setPosition(pos);
            intakeRotateServo.setPosition(0);
            outakeSampleServo.setPosition(0);
            telemetry.addData("position outakeArmServo", outakeArmServo.getPosition());
            telemetry.addData("position intakeRotateServo", intakeRotateServo.getPosition());
            telemetry.addData("position outakeSampleServo", outakeSampleServo.getPosition());
            updateTelemetry(telemetry);
        }
    }
}