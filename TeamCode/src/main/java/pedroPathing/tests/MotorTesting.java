package pedroPathing.tests;

import static pedroPathing.PositionStorage.intakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.PositionStorage.resetStuff;
import static pedroPathing.PositionStorage.servoextended;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import pedroPathing.ControlMotor;
import pedroPathing.Toggle;

@TeleOp(name = "Motor Testing", group = "Linear OpMode")
public class MotorTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outakeRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();
        dashboardTelemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.

        double outakeMotorPower =0;
        intakeTargetPos = 0;
        outakeTargetPos = 0;
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        ControlMotor intakeControlMotor = new ControlMotor();
        ControlMotor outakeControlMotor = new ControlMotor();
        resetStuff();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.a)
                outakeTargetPos = 0;
            if(gamepad1.x)
                outakeTargetPos = 2500;
            if(gamepad1.b)
                outakeTargetPos+=50;
            if(gamepad1.y)
                outakeTargetPos-=50;
            if(gamepad1.dpad_up)
                outakeMotorPower += 0.002;
            if(gamepad1.dpad_down)
                outakeMotorPower -= 0.002;



            if(outakeLeftMotor.getCurrentPosition()+5 < outakeTargetPos)
                outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
            else if((outakeLeftMotor.getCurrentPosition()-5 > outakeTargetPos))
                outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());

            else
                outakeMotorPower =0;

            if(outakeLeftMotor.getCurrentPosition()+46 < outakeTargetPos)
                outakeMotorPower =1;
            else if((outakeLeftMotor.getCurrentPosition()-46 > outakeTargetPos))
                outakeMotorPower =-1;


            if(outakeTargetPos!=0) outakeMotorPower +=0.09;
            outakeRightMotor.setPower(-outakeMotorPower);
            outakeLeftMotor.setPower(outakeMotorPower);

            //telemetry
            telemetry.addData("outake motor pos ", outakeLeftMotor.getCurrentPosition());
            telemetry.addData("intake motor pos ", intakeMotor.getCurrentPosition());
            telemetry.addData("outake motor power ", outakeMotorPower);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            FtcDashboard.getInstance().startCameraStream(portal, 10);
            dashboardTelemetry.update();
            telemetry.update();
        }

    }
}