package pedroPathing.tests;


import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.OrganizedPositionStorage.PIDincrement;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeTargetPosAdder;
import static pedroPathing.newOld.PositionStorage.resetStuff;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.OrganizedPositionStorage;
import pedroPathing.newOld.Toggle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


import pedroPathing.AutoPIDS.NewPidsController;

import pedroPathing.newOld.Toggle;
import static pedroPathing.newOld.PositionStorage.*;

import android.util.Size;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Outtake Test", group = "Linear OpMode")
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
