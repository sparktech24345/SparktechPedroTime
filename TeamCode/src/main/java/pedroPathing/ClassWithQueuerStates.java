package pedroPathing;

import static pedroPathing.OrganizedPositionStorage.*;
import static pedroPathing.ClassWithStates.*;
import pedroPathing.Queuer.*;

public class ClassWithQueuerStates {

    public static Step steperes = new Step(false, new Runnable() {
        public void run() {

        }
    });

    public static Queuer intakeCabinQueue = new Queuer();
    public static Queuer intakeSlidersQueue = new Queuer();
    public static Queuer outtakeQueue = new Queuer();





    //Intake States
    public static void intakeExtended4out4Queue(){

        intakeState = intakeStates.intakeExtended4out4;

        intakeSlidersQueue.clearSteps();

        Step extendIntake4out4 = new Step(true, new Runnable() {
            public void run() {
                intakeExtendMotorTargetPos = 510;
                intakeGravitySubtractor =8;
            }
        });

        intakeSlidersQueue.addStep(extendIntake4out4);
    }


    public static void intakeExtended3out4Queue(){
        intakeState = intakeStates.intakeExtended3out4;

        intakeSlidersQueue.clearSteps();

        Step extendIntake3out4 = new Step(true, new Runnable() {
            public void run() {
                intakeExtendMotorTargetPos = 377;
                intakeGravitySubtractor =6;
            }
        });

        intakeSlidersQueue.addStep(extendIntake3out4);
    }


    public static void intakeExtended2out4Queue(){
        intakeState = intakeStates.intakeExtended2out4;

        intakeSlidersQueue.clearSteps();

        Step extendIntake2out4 = new Step(true, new Runnable() {
            public void run() {
                intakeExtendMotorTargetPos = 245;
                intakeGravitySubtractor =4;
            }
        });

        intakeSlidersQueue.addStep(extendIntake2out4);
    }


    public static void intakeExtended1out4Queue(){
        intakeState = intakeStates.intakeExtended1out4;

        intakeSlidersQueue.clearSteps();

        Step extendIntake1out4 = new Step(true, new Runnable() {
            public void run() {
                intakeExtendMotorTargetPos = 112;
                intakeGravitySubtractor =2;
            }
        });

        intakeSlidersQueue.addStep(extendIntake1out4);
    }


    public static void intakeRetractedQueue(){
        intakeState = intakeStates.intakeRetracted;

        intakeSlidersQueue.clearSteps();

        Step retractIntake = new Step(true, new Runnable() {
            public void run() {
                intakeExtendMotorTargetPos = 0;
                intakeGravitySubtractor =0;
            }
        });

        intakeSlidersQueue.addStep(retractIntake);
    }






    //intake cabin states

    public static void intakeCabinDownCollectingQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = 1;
    }
    public static void intakeCabinDownOutputtingQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinDownOutputting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = -0.5;
    }
    public static void intakeCabinDownStandStillQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinDownStandStill;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinTransferPositionQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinTransferPositionWithPowerQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeSpinMotorPow = 1;
    }
    public static void intakeCabinFullInBotQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBot;
        intakePivotServoPos = intakePivotServoOutputTruBotPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinFullInBotOutputtingQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBotOutputting;
        intakePivotServoPos = intakePivotServoOutputTruBotPos;
        intakeSpinMotorPow = -1;
    }







    //Outtake States


    public static void outtakeSpecimenHangQueue(){
        outtakeState = outtakeStates.outtakeSpecimenHang;
        outtakePivotServoPos = outtakePivotServoHighRungHangPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = outtakeSliderSpecimenHangPos;
    }
    public static void autoOuttakeSpecimenHangQueue(){
        outtakeState = outtakeStates.outtakeSpecimenHang;
        outtakePivotServoPos = outtakePivotServoHighRungHangPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = autoOuttakeSliderSpecimenHangPos;
    }
    public static void outtakeBasketQueue(){
        outtakeState = outtakeStates.outtakeBasket;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakePivotServoPos = outtakePivotServoBasketPos;
        outtakeExtendMotorTargetPos = outtakeMotorMaxPos;
    }
    public static void outtakeWallPickUpNormalQueue(){ //old one
        outtakeState = outtakeStates.outtakeWallPickUpNormal;
        outtakePivotServoPos = 0;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
    }
    public static void outtakeWallPickUpNewQueue(){
        outtakeState = outtakeStates.outtakeWallPickUpNew;
        //outtakePivotServoPos = outtakePivotServoWallPickupPos;  //wire interference
        isInNeedToGoToSpecimenTransferPos = true;
        needsToExtraExtend = true;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        //outtakeIsInNeedToExtraExtendClaw = true;
        outtakeIsInNeedToExtraExtendClawTimer = System.currentTimeMillis();
        outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
    }
    public static void outtakeTransferQueue(){
        outtakeState = outtakeStates.outtakeTransfer;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakePivotServoPos = outtakePivotServoTransferPos;
        //outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
        isOuttakeInPositionToGoDown = true;
        beforeOuttakeGoDownTimer = System.currentTimeMillis();
    }
    public static void outtakeStandByBasketQueue(){
        outtakeState = outtakeStates.outtakeStandBy;
        outtakePivotServoPos = outtakePivotServoStandByPos;
        outtakeExtendMotorTargetPos = outtakeMotorStandByPos;
    }

    public static void outtakeStandByWithoutExtensionsQueue(){
        outtakeState = outtakeStates.outtakeStandByWithoutExtensions;
        outtakePivotServoPos = outtakePivotServoStandByPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
    }


}
