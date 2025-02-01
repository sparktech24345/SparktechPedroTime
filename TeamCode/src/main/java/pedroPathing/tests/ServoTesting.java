package pedroPathing.tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.ControlMotor;

import pedroPathing.Toggle;
import static pedroPathing.PositionStorage.*;

@TeleOp(name = "ServoTesting", group = "Linear OpMode")
public class ServoTesting extends LinearOpMode {
    double intakeRotateServoPosition = 30;
    double outakeArmServoPosition = 60;
    double outakeSampleServoPosition = servoextended;
    //double outakeRotateServoPosition =161; // new 0
    double intakeServoPower = 0;

    SparkFunOTOS myOtos;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        //Servo outakeRotateServo = hardwareMap.get(Servo.class, "outakeRotateServo");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        ControlMotor intakeControlMotor = new ControlMotor();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper)
                outakeArmServoPosition += 0.2;
            if (gamepad1.left_bumper)
                outakeArmServoPosition -= 0.2;
            if(gamepad1.a)
                outakeSampleServoPosition--;
            if(gamepad1.b)
                outakeSampleServoPosition++;
            if(gamepad1.x)
                intakeRotateServoPosition+=0.2;
            if(gamepad1.y)
                intakeRotateServoPosition-=0.2;//*/
// 64 deschis 0 inchis la sample
// 0 la rotate 60 pozitie baschet



            if (Toggle.outputtoggle(gamepad1.right_trigger > 0) != 0)
                intakeServoPower = Toggle.outputtoggle(gamepad1.right_trigger > 0);

            //telemetry
            telemetry.addData("outakeArmServoPOS GO TO", outakeArmServoPosition);
            telemetry.addData("outakeArmServoPOS", outakeArmServo.getPosition());
            telemetry.addData("outakeSamplePOS", outakeSampleServo.getPosition());
            telemetry.addData("outakeSamplePOS GO TO ", outakeSampleServoPosition);
            telemetry.addData("intakeRotateServoPosition", intakeRotateServoPosition);
            telemetry.addData("intakeRotateServoPos(TBS)", intakeRotateServo.getPosition());
            telemetry.addData("outake motor pos ", outakeLeftMotor.getCurrentPosition());
            telemetry.addData("intake motor pos ", intakeMotor.getCurrentPosition());
            telemetry.update();



            //tester.setPosition(0);
            intakeRotateServo.setPosition(intakeRotateServoPosition / 360);
            outakeArmServo.setPosition(outakeArmServoPosition / 360);
            outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
            //outakeRotateServo.setPosition(outakeRotateServoPosition/360);


        }

    }
}