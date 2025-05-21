package pedroPathing;


import static pedroPathing.ClassWithQueuerStates.*;
import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
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

        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
                outtakeTransferQueue(); //includes going down
                shouldTransfer = false;
            }
            if(shouldSpecimenTransfer){
                intakeCabinSpecimenTransferQueue();
                intakeRetracted();
                outtakeSpecimenHangQueue();
                shouldSpecimenTransfer = false;
            }

            isPressedX1v2 = gamepad1.x;

            //Basket
            if(gamepad1.x) isPressedX1 = true;
            if(gamepad1.x && outtakeState == outtakeStates.outtakeBasket){
                outtakeGetDownFromBasket();
                isPressedX1 = false;
            }
            if(!gamepad1.x && isPressedX1 && !(outtakeState == outtakeStates.outtakeBasket)){

                if(outtakeState == outtakeStates.outtakeSpecimenHang || outtakeState == outtakeStates.outtakeTransfer) outtakeBasketQueue();
                else{ /*if(gamepad2.b)*/ outtakeBasketQueue();}

                isPressedX1 = false;
            }

            //Specimen

            if(gamepad1.b) isPressedB1 = true;
            if(!gamepad1.b && isPressedB1){

                if(outtakeState == outtakeStates.outtakeSpecimenHang) outtakeWallPickUpNewQueue();
                else if(outtakeState == outtakeStates.outtakeWallPickUpNew) outtakeSpecimenHangQueue();
                else outtakeSpecimenHangQueue();

                isPressedB1 = false;
            }

            if(gamepad1.y){
                if(intakeCabinState == intakeCabinStates.intakeCabinFullInBotOutputting){wasIntakeCabinTruBotOutputting = true;}
                else if(isIntakeInPositionToOutputTruBot && robot.intake.intakeMotor.getCurrentPosition() < 10 && outtakeState == outtakeStates.outtakeSpecimenHang){
                    isIntakeInPositionToOutputTruBot = false;
                    intakeCabinFullInBotOutputting();
                }
                else{
                    intakeRetractedQueue();
                    intakeCabinFullInBotQueue();
                    outtakeSpecimenHangQueue();
                }
            }
            else if(wasIntakeCabinTruBotOutputting){
                wasIntakeCabinTruBotOutputting = false;
                intakeCabinFullInBot();
            }

            if(gamepad1.right_bumper){
                for(int i=0; i<10;i++){
                    outtakeExtended4out4Queue();
                    outtakeExtended1out4Queue();
                }
            }


            //Intake positions
            if (gamepad1.dpad_left)  intakeExtended4out4Queue();
            if (gamepad1.dpad_down)  intakeExtended3out4Queue();
            if (gamepad1.dpad_right) intakeExtended2out4Queue();
            if(gamepad1.dpad_up)     intakeExtended1out4Queue();
            if(gamepad2.left_bumper) intakeRetractedQueue();


            //if (gamepad1.dpad_left)  outtakeExtended4out4Queue();
            //if (gamepad1.dpad_down)  outtakeExtended3out4Queue();
            //if (gamepad1.dpad_right) outtakeExtended2out4Queue();
            //if(gamepad1.dpad_up)     outtakeExtended1out4Queue();




            //do it 5 times cuz im dumb and we must do checking stuff for every new step even thoough we checked before
            //for(int i=0; i<5 && opModeIsActive(); i++) {
            //    updateTelemetry(tel);
            intakeCabinQueue.updateSteps(tel);
            intakeSlidersQueue.updateSteps(tel);
            outtakeQueue.updateSteps(tel);
            //}

            robot.robotSetPower();


            tel.addData("intakeSliderState",intakeState);
            tel.addData("intakeCabinState",intakeCabinState);
            tel.addData("outtakeState",outtakeState);
            tel.addData("color stuff",currentStateOfSampleInIntake);
            tel.addData("outakeArmServoPOS GO TO", outtakePivotServoPos);
            tel.addData("outakeSamplePOS GO TO ", outtakeClawServoPos);
            tel.addData("intakeRotateServoPosition", intakePivotServoPos);
            tel.addData("intakeExtendMotorPow",robot.intake.intakeMotor.getPower());
            tel.addData("outakeMotorPow",robot.outtake.outakeLeftMotor.getPower());
            tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
            tel.addData("intake current pos",tempIntakeAPosition);
            tel.addData("outtake current pos",tempOuttakeAPosition);
            tel.addData("blue color",robot.colors.blue);
            tel.addData("red color",robot.colors.red);

            updateTelemetry(tel);
        }

    }


}