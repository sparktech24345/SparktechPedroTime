package pedroPathing;


import static pedroPathing.ClassWithQueuerStates.*;
import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import pedroPathing.SubSys.OurRobot;
import pedroPathing.newOld.Toggle;


@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "BBBNewStatesOutputQueue", group = "Linear OpMode")
public class NewStateyBBBstyleOutputQueueTime extends LinearOpMode {
    MultipleTelemetry tel;


    @Override
    public void runOpMode() throws InterruptedException {


        OrganizedPositionStorage.resetStuff();
        OurRobot robot = new OurRobot(hardwareMap,tel,gamepad1,gamepad2);
        robot.initRobot();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while (opModeIsActive()) {


            robot.doConstantStuff();

            ///CONTROLS


            //COLLECTING
            if(gamepad1.a) isPressedA1 = true;
            if(!gamepad1.a && isPressedA1){


                if(intakeCabinState == intakeCabinStates.intakeCabinDownCollecting || intakeCabinState == intakeCabinStates.isInTransferToIntakeCabinDownCollecting){
                    if(!isInSpecimenState) shouldTransfer = true;
                    else shouldSpecimenTransfer = true;
                }
                else{
                    intakeCabinDownCollectingQueue();
                }
                isPressedA1 = false;
            }


            //TRANSFER
            if(shouldTransfer){
                intakeCabinTransferQueue();
                intakeRetracted();
                outtakeTransferQueue();
                shouldTransfer = false;
            }
            if(shouldSpecimenTransfer){
                intakeCabinSpecimenTransferQueue();
                intakeRetracted();
                outtakeSpecimenHangQueue();
                shouldSpecimenTransfer = false;
            }



            //Basket
            if(gamepad1.x) isPressedX1 = true;
            if(!gamepad1.x && isPressedX1){

                if(outtakeState == outtakeStates.outtakeSpecimenHang || outtakeState == outtakeStates.outtakeTransfer) outtakeBasketQueue();
                if(outtakeState == outtakeStates.outtakeBasket) outtakeTransfer();
                if(gamepad2.b) outtakeBasketQueue();

                isPressedX1 = false;
            }

            //Specimen

            if(gamepad1.b) isPressedB1 = true;
            if(!gamepad1.b && isPressedB1){

                isPressedB1 = false;
            }



            //Intake positions
            if (gamepad1.dpad_left)  intakeExtended4out4();
            if (gamepad1.dpad_down)  intakeExtended3out4();
            if (gamepad1.dpad_right) intakeExtended2out4();
            if(gamepad1.dpad_up)     intakeExtended1out4();
            if(gamepad2.left_bumper) intakeRetracted();




            robot.robotSetPower();


            telemetry.addData("intakeSliderState",intakeState);
            telemetry.addData("intakeCabinState",intakeCabinState);
            telemetry.addData("outtakeState",outtakeState);
            telemetry.addData("color stuff",currentStateOfSampleInIntake);
            telemetry.addData("outakeArmServoPOS GO TO", outtakePivotServoPos);
            telemetry.addData("outakeSamplePOS GO TO ", outtakeClawServoPos);
            telemetry.addData("intakeRotateServoPosition", intakePivotServoPos);
            telemetry.addData("intakeExtendMotorPow",robot.intake.intakeMotor.getPowerFloat());
            telemetry.addData("outakeMotorPow",robot.outtake.outakeLeftMotor.getPowerFloat());
            telemetry.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            telemetry.addData("outtake current pos",robot.outtake.outakeLeftMotor.getCurrentPosition());
            telemetry.addData("blue color",robot.colors.blue);
            telemetry.addData("red color",robot.colors.red);

            updateTelemetry(telemetry);
        }

    }

    private void intakeCabinSpecimenTransferQueue() {
    }

    private void intakeCabinTransferQueue() {
    }

}