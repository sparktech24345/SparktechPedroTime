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
        intakeCabinQueue.clearSteps();


        if(!(tempOuttakeAPosition > 400) && (intakeCabinState == intakeCabinStates.intakeCabinFullInBot && (intakeState==intakeStates.intakeRetracted || intakeState == intakeStates.intakeExtended1out4))){

            Step getOutOfWay = new Step(true, new Runnable() {
                public void run() {
                    outtakeExtended1out4Queue();
                }
            });


            Step intakeDownWithPower = new Step(tempOuttakeAPosition > 400, new Runnable() {
                public void run() {
                    intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
                    intakePivotServoPos = intakePivotServoPickupPos;
                    intakeSpinMotorPow = 1;
                    outtakeRetractedQueue();
                }
            });



            intakeCabinQueue.addStep(getOutOfWay);
            intakeCabinQueue.addStep(intakeDownWithPower);
        }
        else {
            Step intakeDownWithPower = new Step(true, new Runnable() {
                public void run() {
                    intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
                    intakePivotServoPos = intakePivotServoPickupPos;
                    intakeSpinMotorPow = 1;
                }
            });



            intakeCabinQueue.addStep(intakeDownWithPower);
        }


    }
    public static void intakeCabinDownOutputtingQueue(){
        intakeCabinState = intakeCabinStates.intakeCabinDownOutputting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = -1;
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

    public static void intakeCabinTransferQueue() {
        intakeCabinQueue.clearSteps();

        Step intakeUpWithPower = new Step(true, new Runnable() {
            public void run() {
                intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
                intakePivotServoPos = intakePivotServoTransferPos;
                //shouldStopIntakeCabinSpinningAfterTakig = true;
                shouldStopIntakeCabinSpinningAfterTakigTimer = System.currentTimeMillis();
                intakeSpinMotorPow = 1;
            }
        });

        Step intakeTransferPosWithoutPower = new Step(shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
                intakePivotServoPos = intakePivotServoTransferPos;
                intakeSpinMotorPow = 0;
            }
        });

        if(intakeCabinState == intakeCabinStates.isInTransferToIntakeCabinDownCollecting) intakeCabinQueue.addStep(intakeUpWithPower);
        intakeCabinQueue.addStep(intakeTransferPosWithoutPower);
    }


    public static void intakeCabinSpecimenTransferQueue() {
        intakeCabinQueue.clearSteps();

        Step intakeUpWithPower = new Step(true, new Runnable() {
            public void run() {
                intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
                intakePivotServoPos = intakePivotServoTransferPos;
                //shouldStopIntakeCabinSpinningAfterTakig = true;
                shouldStopIntakeCabinSpinningAfterTakigTimer = System.currentTimeMillis();
                intakeSpinMotorPow = 1;
            }
        });

        Step intakeTransferPosWithoutPower = new Step(shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                intakeCabinState = intakeCabinStates.intakeCabinFullInBot;
                intakePivotServoPos = intakePivotServoOutputTruBotPos;
                intakeSpinMotorPow = 0;
            }
        });

        if(intakeCabinState == intakeCabinStates.isInTransferToIntakeCabinDownCollecting) intakeCabinQueue.addStep(intakeUpWithPower);
        intakeCabinQueue.addStep(intakeTransferPosWithoutPower);
    }






    //Outtake States


    public static void outtakeSpecimenHangQueue(){
        outtakeQueue.clearSteps();

        Step closeClaw = new Step(true, new Runnable() {
            public void run() {
                outtakePivotServoPos = outtakeClawServoRetractedPos;
                outtakeCloseClawInTransferTimer = System.currentTimeMillis();
            }
        });

        Step goUp = new Step(outtakeCloseClawInTransferTimer + 200 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                if(isRobotInAuto) outtakeExtendMotorTargetPos = autoOuttakeSliderSpecimenHangPos;
                else outtakeExtendMotorTargetPos = outtakeSliderSpecimenHangPos;
            }
        });

        Step doRestOfActions = new Step(/*tempOuttakeAPosition > 500*/true, new Runnable() {
            public void run() {
                outtakeState = outtakeStates.outtakeSpecimenHang;
                outtakePivotServoPos = outtakePivotServoHighRungHangPos;
            }
        });

        outtakeQueue.addStep(closeClaw);
        outtakeQueue.addStep(goUp);
        outtakeQueue.addStep(doRestOfActions);
    }




    public static void outtakeBasketQueue(){
        outtakeQueue.clearSteps();
        outtakeState = outtakeStates.outtakeBasket;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakePivotServoPos = outtakePivotServoBasketPos;
        outtakeExtendMotorTargetPos = outtakeMotorMaxPos;
    }



    public static void outtakeWallPickUpNewQueue(){
        outtakeQueue.clearSteps();


        Step goUp = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
                outtakeClawServoPos = outtakeClawServoExtendedPos;
            }
        });

        Step doRestOfActions = new Step(/*tempOuttakeAPosition > 500*/true, new Runnable() {
            public void run() {
                outtakeState = outtakeStates.outtakeWallPickUpNew;
                outtakePivotServoPos = outtakePivotServoWallPickupPos;
                outtakeIsInNeedToExtraExtendClawTimer = System.currentTimeMillis();
            }
        });

        Step extraOpenClaw = new Step(outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                outtakePivotServoPos = outtakeClawServoExtraExtendedPos;
            }
        });

        outtakeQueue.addStep(goUp);
        outtakeQueue.addStep(doRestOfActions);
        outtakeQueue.addStep(extraOpenClaw);
    }




    public static void outtakeTransferQueue(){
        outtakeQueue.clearSteps();

        Step PrepareServoForTransfer = new Step(true, new Runnable() {
            public void run() {
                outtakeClawServoPos = outtakeClawServoExtendedPos;
                outtakePivotServoPos = outtakePivotServoTransferPos;
                beforeOuttakeGoDownTimer = System.currentTimeMillis();
            }
        });

        Step PrepareSlidersForTransfer = new Step((!(outtakeState == outtakeStates.outtakeWallPickUpNew)) || beforeOuttakeGoDownTimer + 400 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
            }
        });

        Step closeClaw = new Step(tempIntakeAPosition < 8 && shouldStopIntakeCabinSpinningAfterTakigTimer + 600 < System.currentTimeMillis()  && tempOuttakeAPosition < 25, new Runnable() {
            public void run() {
                outtakePivotServoPos = outtakeClawServoRetractedPos;
                outtakeCloseClawInTransferTimer = System.currentTimeMillis();
            }
        });

        Step doRestOfActions = new Step(outtakeCloseClawInTransferTimer + 200 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                outtakeState = outtakeStates.outtakeSpecimenHang;
                outtakePivotServoPos = outtakePivotServoHighRungHangPos;
                //outtakeClawServoPos = outtakeClawServoRetractedPos;
                outtakeExtendMotorTargetPos = outtakeSliderSpecimenHangPos;
            }
        });

        outtakeQueue.addStep(PrepareServoForTransfer);
        outtakeQueue.addStep(PrepareSlidersForTransfer);
        outtakeQueue.addStep(closeClaw);
        outtakeQueue.addStep(doRestOfActions);
    }




    public static void outtakeGetDownFromBasket(){
        outtakeQueue.clearSteps();

        Step openServo = new Step(true, new Runnable() {
            public void run() {
                outtakeClawServoPos = outtakeClawServoExtendedPos;
                outtakeAfterBasketSampleScoreTimer = System.currentTimeMillis();
            }
        });

        Step goDown = new Step(!isPressedX1v2 && outtakeAfterBasketSampleScoreTimer + 300 < System.currentTimeMillis(), new Runnable() {
            public void run() {
                outtakeState = outtakeStates.outtakeTransfer;
                outtakePivotServoPos = outtakePivotServoTransferPos;
                outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
            }
        });

        outtakeQueue.addStep(openServo);
        outtakeQueue.addStep(goDown);
    }




    public static void outtakeExtended4out4Queue(){
        outtakeQueue.clearSteps();

        Step extendIntake4out4 = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = 2000;
            }
        });

        outtakeQueue.addStep(extendIntake4out4);
    }


    public static void outtakeExtended3out4Queue(){
        outtakeQueue.clearSteps();

        Step extendIntake3out4 = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = 1500;
            }
        });

        outtakeQueue.addStep(extendIntake3out4);
    }


    public static void outtakeExtended2out4Queue(){
        outtakeQueue.clearSteps();

        Step extendIntake2out4 = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = 1000;
            }
        });

        outtakeQueue.addStep(extendIntake2out4);
    }


    public static void outtakeExtended1out4Queue(){
        outtakeQueue.clearSteps();

        Step extendIntake1out4 = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = 500;
            }
        });

        outtakeQueue.addStep(extendIntake1out4);
    }
    public static void outtakeRetractedQueue(){
        outtakeQueue.clearSteps();

        Step extendIntake1out4 = new Step(true, new Runnable() {
            public void run() {
                outtakeExtendMotorTargetPos = 0;
            }
        });

        outtakeQueue.addStep(extendIntake1out4);
    }















}
