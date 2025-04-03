package pedroPathing.tests;


import static pedroPathing.newOld.PositionStorage.backLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.backRightPowerCat;
import static pedroPathing.newOld.PositionStorage.frontLeftPowerCat;
import static pedroPathing.newOld.PositionStorage.frontRightPowerCat;
import static pedroPathing.newOld.PositionStorage.intakeActualZero;
import static pedroPathing.newOld.PositionStorage.intakeTargetPos;
import static pedroPathing.newOld.PositionStorage.intakeTargetPosAdder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.newOld.ControlMotor;

@TeleOp(name = "Only Drive Teleop", group = "Linear OpMode")
@Disabled
public class DriveOnlyTest extends LinearOpMode {
    ExecutorService executorService = Executors.newFixedThreadPool(2);

    ControlMotor intakeControlMotor = new ControlMotor();

    @Override
    public void runOpMode() throws InterruptedException {

       /* DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/


        if (isStopRequested()) return;
        waitForStart();

        executorService.execute(new Runnable() {

            @Override
            public void run() {
                DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
                DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
                DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
                DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
                DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/

                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while(opModeIsActive()){
                    double intakeMotorPower = 0;
                    intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeActualZero+intakeTargetPosAdder, intakeMotor.getCurrentPosition());

                    frontLeftMotor.setPower(frontLeftPowerCat);
                    backLeftMotor.setPower(backLeftPowerCat);
                    frontRightMotor.setPower(frontRightPowerCat);
                    backRightMotor.setPower(backRightPowerCat);
                    intakeMotor.setPower(intakeMotorPower);//*/
                    telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    updateTelemetry(telemetry);
                }
//*/
            }
        });
        while (opModeIsActive()) {

            // executorService.submit(Motors::new);
            intakeTargetPos = 0;


            ///gamepad1
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            boolean slowdown = gamepad1.left_bumper;

            //calculating nedded power by method 1
            frontRightPowerCat = (pivot - vertical - horizontal);
            backRightPowerCat = (pivot - vertical + horizontal);
            frontLeftPowerCat = (pivot + vertical - horizontal);
            backLeftPowerCat = (pivot + vertical + horizontal);

            //Pivot stuff
            pivot = -pivot;


            if(slowdown) {
                frontRightPowerCat /= 3;
                backRightPowerCat /= 3;
                frontLeftPowerCat /= 3;
                backLeftPowerCat /= 3;
            }

            telemetry.addData("frontLeftPower",frontLeftPowerCat);
            telemetry.addData("backLeftPowerCat",backLeftPowerCat);
            telemetry.addData("frontRightPowerCat",frontRightPowerCat);
            telemetry.addData("backRightPowerCat",backRightPowerCat);
            telemetry.addLine("This is Main "+Thread.currentThread().getId());
            updateTelemetry(telemetry);

        }

    }
}