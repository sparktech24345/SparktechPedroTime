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

        updateTelemetry(telemetry);
        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("is active",true);
            updateTelemetry(telemetry);
        }

    }
}
