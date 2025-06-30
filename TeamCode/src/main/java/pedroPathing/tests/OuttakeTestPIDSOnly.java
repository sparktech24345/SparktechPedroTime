package pedroPathing.tests;


import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import pedroPathing.AutoPIDS.NewPidsController;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Outtake Test", group = "Linear OpMode")
@Disabled
public class OuttakeTestPIDSOnly extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");


        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.b){
                outtakeExtendMotorTargetPos = 2000;
            }

            if(gamepad1.a){
                outtakeExtendMotorTargetPos = 0;
            }

            if(gamepad1.x){
                outtakeExtendMotorTargetPos = 1000;
            }

            double outtakeMotorCurrentPos = outakeLeftMotor.getCurrentPosition();

            //PID OUTTAKE
            double outtakeMotorPower;
            outtakeMotorPower = NewPidsController.pidControllerOuttake(outtakeExtendMotorTargetPos, outtakeMotorCurrentPos);

            outakeRightMotor.setPower(outtakeMotorPower);
            outakeLeftMotor.setPower(outtakeMotorPower);

            telemetry.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            telemetry.addData("outtake current pos",outtakeMotorCurrentPos);
            updateTelemetry(telemetry);

        }

    }


}
