package pedroPathing;

import static pedroPathing.OrganizedPositionStorage.*;
public class ClassWithStates {

    //state holders

    //**************************************************************************\\

    // intake
    public static enum intakeStates{
        noStateSet,
        intakeExtended4out4,
        intakeExtended3out4,
        intakeExtended2out4,
        intakeExtended1out4,
        intakeRetracted,
    }
    public static intakeStates intakeState = intakeStates.noStateSet;

    //*************************************************************************\\

    //intake cabin

    public static enum intakeCabinStates{
        noStateSet,
        intakeCabinDownCollecting,
        intakeCabinDownOutputting,
        intakeCabinDownStandStill,
        intakeCabinTransferPosition,
        intakeCabinFullInBot,
        intakeCabinFullInBotOutputting,
    }
    public static intakeCabinStates intakeCabinState = intakeCabinStates.noStateSet;

    //**************************************************************************\\

    //outtake

    public static enum outtakeStates{
        noStateSet,
        outtakeSpecimenHang,
        outtakeBasket,
        outtakeWallPickUpNormal,
        outtakeWallPickUpNew,
        outtakeTransfer,
        outtakeStandBy,
    }
    public static outtakeStates outtakeState = outtakeStates.noStateSet;

    //***************************************************************************\\









    //Intake States

    public static void intakeExtended4out4(){
        intakeState = intakeStates.intakeExtended4out4;
        intakeTargetPos = 510;
    }
    public static void intakeExtended3out4(){
        intakeState = intakeStates.intakeExtended3out4;
        intakeTargetPos = 377;
    }
    public static void intakeExtended2out4(){
        intakeState = intakeStates.intakeExtended2out4;
        intakeTargetPos = 245;
    }
    public static void intakeExtended1out4(){
        intakeState = intakeStates.intakeExtended1out4;
        intakeTargetPos = 112;
    }
    public static void intakeRetracted(){
        intakeState = intakeStates.intakeRetracted;
        intakeTargetPos = 112;
    }






    //intake cabin states

    public static void intakeCabinDownCollecting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
    }
    public static void intakeCabinDownOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownOutputting;
    }
    public static void intakeCabinDownStandStill(){
        intakeCabinState = intakeCabinStates.intakeCabinDownStandStill;
    }
    public static void intakeCabinTransferPosition(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
    }
    public static void intakeCabinFullInBot(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBot;
    }
    public static void intakeCabinFullInBotOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBotOutputting;
    }






    //Outtake States


    public static void outtakeSpecimenHang(){
        outtakeState = outtakeStates.outtakeSpecimenHang;
    }
    public static void outtakeBasket(){
        outtakeState = outtakeStates.outtakeBasket;
    }
    public static void outtakeWallPickUpNormal(){
        outtakeState = outtakeStates.outtakeWallPickUpNormal;
    }
    public static void outtakeWallPickUpNew(){
        outtakeState = outtakeStates.outtakeWallPickUpNew;
    }
    public static void outtakeTransfer(){
        outtakeState = outtakeStates.outtakeTransfer;
    }
    public static void outtakeStandBy(){
        outtakeState = outtakeStates.outtakeStandBy;
    }



    //init method cuz why not

    public static void initStates(){
        outtakeStandBy();
        intakeCabinFullInBot();
        intakeRetracted();
    }

}
