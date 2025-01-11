package pedroPathing.tests;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static pedroPathing.States.PositionStorage.backLeftPowerCat;
import static pedroPathing.States.PositionStorage.backRightPowerCat;
import static pedroPathing.States.PositionStorage.frontLeftPowerCat;
import static pedroPathing.States.PositionStorage.frontRightPowerCat;
import static pedroPathing.States.PositionStorage.intakeTargetPos;
import static pedroPathing.States.PositionStorage.outakeTargetPos;

import android.provider.Telephony;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.reflect.Executable;
import java.util.Objects;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.ControlMotor;
import pedroPathing.TempRunnable;

@TeleOp(name = "Empty Teleop", group = "Linear OpMode")
public class EmptyTeleop extends LinearOpMode {
    volatile boolean keepRunning = true;


    @Override
    public void runOpMode() throws InterruptedException {
        ExecutorService executorService = Executors.newFixedThreadPool(4);
        executorService.submit(Motors::new);
        updateTelemetry(telemetry);
        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("is active",true);
            updateTelemetry(telemetry);
        }

    }
}
class Motors implements Callable<Object>{

    double intakeMotorPower;
    double outakeMotorPower;

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    private boolean DoOnce = true;
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
    DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
    DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
    DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");

    public Motors() {
        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();
    }

    @Override
    public Object call() throws Exception {


            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DoOnce = false;
                frontLeftMotor.setPower(frontLeftPowerCat);
                backLeftMotor.setPower(backLeftPowerCat);
                frontRightMotor.setPower(frontRightPowerCat);
                backRightMotor.setPower(backRightPowerCat);//*/
                intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos, intakeMotor.getCurrentPosition());
                outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos, outakeRightMotor.getCurrentPosition());
                intakeMotor.setPower(intakeMotorPower);
                outakeRightMotor.setPower(outakeMotorPower);
                outakeLeftMotor.setPower(outakeMotorPower);
                telemetry.addData("Hello World", true);
                telemetry.update();
                Thread.sleep(15);

            return null;
    }

}
class Main implements Callable<Object>{

    @Override
    public Object call() throws Exception {
        return null;
    }

}