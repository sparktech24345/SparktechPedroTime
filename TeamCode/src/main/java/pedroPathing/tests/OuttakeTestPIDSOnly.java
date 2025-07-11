package pedroPathing.tests;


import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import pedroPathing.PIDStorageAndUse.NewPidsController;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Outtake Test", group = "Linear OpMode")

public class OuttakeTestPIDSOnly extends LinearOpMode{
    MultipleTelemetry tel;
    public void runOpMode() throws InterruptedException {
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");

        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        outakeRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/

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

            tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            tel.addData("outtake current pos",outtakeMotorCurrentPos);
            tel.addData("motor power ",outtakeMotorPower);
            tel.update();

        }

    }


}
