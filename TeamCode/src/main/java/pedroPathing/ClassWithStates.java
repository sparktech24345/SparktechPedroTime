package pedroPathing;

import static pedroPathing.OrganizedPositionStorage.*;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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

    public static enum colorSensorOutty{
        noSample,
        correctSample,
        wrongSample,
    }

    public static colorSensorOutty currentStateOfSampleInIntake = colorSensorOutty.noSample;

    //***************************************************************************\\

    public static enum colorList{
        teamNotSet,
        red,
        blue,
        yellow,
    }

    public static colorList currentTeam = colorList.teamNotSet;





    //Intake States

    public static void intakeExtended4out4(){
        intakeState = intakeStates.intakeExtended4out4;
        intakeExtendMotorTargetPos = 510;
    }
    public static void intakeExtended3out4(){
        intakeState = intakeStates.intakeExtended3out4;
        intakeExtendMotorTargetPos = 377;
    }
    public static void intakeExtended2out4(){
        intakeState = intakeStates.intakeExtended2out4;
        intakeExtendMotorTargetPos = 245;
    }
    public static void intakeExtended1out4(){
        intakeState = intakeStates.intakeExtended1out4;
        intakeExtendMotorTargetPos = 112;
    }
    public static void intakeRetracted(){
        intakeState = intakeStates.intakeRetracted;
        intakeExtendMotorTargetPos = 0;
    }






    //intake cabin states

    public static void intakeCabinDownCollecting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownCollecting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = 1;
    }
    public static void intakeCabinDownOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinDownOutputting;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = -0.5;
    }
    public static void intakeCabinDownStandStill(){
        intakeCabinState = intakeCabinStates.intakeCabinDownStandStill;
        intakePivotServoPos = intakePivotServoPickupPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinTransferPosition(){
        intakeCabinState = intakeCabinStates.intakeCabinTransferPosition;
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinFullInBot(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBot;
        intakePivotServoPos = intakePivotServoTransferPos;
        intakeSpinMotorPow = 0;
    }
    public static void intakeCabinFullInBotOutputting(){
        intakeCabinState = intakeCabinStates.intakeCabinFullInBotOutputting;
        intakePivotServoPos = intakePivotServoOutputTruBotPos;
        intakeSpinMotorPow = -1;
    }






    //Outtake States


    public static void outtakeSpecimenHang(){
        outtakeState = outtakeStates.outtakeSpecimenHang;
        outtakePivotServoPos = outtakePivotServoHighRungHangPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = outtakeSliderSpecimenHangPos;
    }
    public static void outtakeBasket(){
        outtakeState = outtakeStates.outtakeBasket;
        outtakePivotServoPos = outtakePivotServoBasketPos;
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        outtakeExtendMotorTargetPos = outtakeMotorMaxPos;
    }
    public static void outtakeWallPickUpNormal(){ //old one
        outtakeState = outtakeStates.outtakeWallPickUpNormal;
        outtakePivotServoPos = 0;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
    }
    public static void outtakeWallPickUpNew(){
        outtakeState = outtakeStates.outtakeWallPickUpNew;
        //outtakePivotServoPos = outtakePivotServoWallPickupPos;  //wire interference
        isInNeedToGoToSpecimenTransferPos = true;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakeExtendMotorTargetPos = outtakeSlidersWallPickPos;
    }
    public static void outtakeTransfer(){
        outtakeState = outtakeStates.outtakeTransfer;
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        outtakePivotServoPos = outtakePivotServoTransferPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
    }
    public static void outtakeStandBy(){
        outtakeState = outtakeStates.outtakeStandBy;
        outtakePivotServoPos = outtakePivotServoStandByPos;
        outtakeExtendMotorTargetPos = outtakeMotorActualZeroPos;
        //TO BE MEASURED
    }

    public static colorSensorOutty ColorCompare(NormalizedRGBA colors, colorList currentTeam,boolean isYellowSampleNotGood){

        if(!(colors.red >= 0.0015 || colors.blue >= 0.0015)) return colorSensorOutty.noSample;
        colorList color=colorList.teamNotSet;

        if (colors.red > colors.blue && colors.red > colors.green)
            color = colorList.red;
        if (colors.blue > colors.red && colors.blue > colors.green)
            color = colorList.blue;
        if (colors.green > colors.blue && colors.green > colors.red)
            color = colorList.yellow;


        //reversing team
        colorList wrongSampleType = colorList.teamNotSet;
        if(currentTeam == colorList.blue) wrongSampleType = colorList.red;
        if(currentTeam == colorList.red) wrongSampleType = colorList.blue;

        if(color == wrongSampleType) return colorSensorOutty.wrongSample;
        if(isYellowSampleNotGood && color == colorList.yellow) return colorSensorOutty.wrongSample;
        else return colorSensorOutty.correctSample;
    }


    //init method cuz why not
    public static void initStates() {
        outtakeStandBy();
        intakeCabinFullInBot();
        intakeRetracted();
    }
}
