package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import pedroPathing.constants.FConstantsForBasket;
import pedroPathing.constants.LConstants;

//@Config
//@Autonomous(name = "Basket Auto RED ALLIANCE Mateis (nu va exploda)", group = "Examples")
public class AutoForBasketMainFile extends OpMode {
    /// FOARTE IMPORTANT: CULOAREA ALIANTEI!
    private colorList teamColor = colorList.red;

    public AutoForBasketMainFile(colorList color){
        this.teamColor = color;
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    final float[] hsvValues = new float[3];
    private DcMotor intakeMotor;
    private DcMotor outakeLeftMotor;
    private DcMotor outakeRightMotor;
    private DcMotor intakeSpinMotor;
    private Servo intakeRotateServo;
    private Servo outakeArmServo;
    private Servo outakeSampleServo;
    private NormalizedColorSensor colorSensor;


    /**                         Our Paths!                          */
    private int pathState = 0;

    double intakeMotorPower=0;
    double outakeMotorPower=0;

    // BASKET
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90)); //start

    // place in basket
    public final double basketY = 38.24 - 1.5;
    public final double basketX = 17.68 - 1.3;

    double basketHeading = 153.86;
    private final Pose behindBasketPreload = new Pose(basketX + 0.3 - 1, basketY - 2, Math.toRadians(basketHeading));
    private final Pose behindBasketFirstSample = new Pose(basketX, basketY, Math.toRadians(basketHeading));
    private final Pose behindBasketSecondSample = new Pose(basketX, basketY, Math.toRadians(basketHeading));
    private final Pose behindBasketThirdSample = new Pose(basketX, basketY , Math.toRadians(basketHeading));

    // collect first 3 from floor
    private final Pose firstSampleCollect = new Pose( 16.69 + 2, 36.92, Math.toRadians(158.69));
    private final Pose secondSampleCollect = new Pose(20.25 + 2, 37.95, Math.toRadians(175.76));
    private final Pose thirdSampleCollect = new Pose( 30.56 - 1.5, 36.74, Math.toRadians(208.27));

    // collect from submersible

    // first
    private final Pose firstSubmersibleStdCollect = new Pose(53, 7.86+0.5, Math.toRadians(90.64));
    private final Pose firstSubmersibleStdBehindBasket = new Pose(17.40 - 2, 35.36 + 2, Math.toRadians(145.66));

    // second
    private final Pose secondSubmersibleStdCollect = new Pose(53, 7.86+0.5, Math.toRadians(90.64));
    private final Pose secondSubmersibleStdBehindBasket = new Pose(17.40 - 1, 35.36 + 1, Math.toRadians(145.66));
    // sign off
    private final Pose endingPosition = new Pose(53, 7.86, Math.toRadians(90.64));

    private final Pose secondTrySub = new Pose(53, 7.86+0.5 + 1, Math.toRadians(90.64));

    private final Point intermediaryExtraPoseForCurves = new Point( 51.35 ,20.48);

    /*
    heading: 4.0289046655823935
intakerotate: 40.0
path state: 0
x: -52.47229868971458
y: 83.09612604576769
slides pos on descent -223
     */

    private boolean skipBasket = false;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    //private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    //private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;

    private PathChain firstSampleCollectPath, preloadScorePath, firstSampleScorePath, secondSampleCollectPath, secondSampleScorePath, thirdSampleCollectPath, thirdSampleScorePath
            , firstSubmersibleCollectPath, firstSubmersibleScorePath,
            secondSubmersibleCollectPath, secondSubmersibleScorePath
            ,secondTrySubPath, signOffPath;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPosePreloadSpecimen)));
//        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPosePreloadSpecimen.getHeading());

        preloadScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(behindBasketPreload)))
                .setLinearHeadingInterpolation(startPose.getHeading(), behindBasketPreload.getHeading())
                .build();

        /// FIRST SAMPLE 

        firstSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindBasketPreload), new Point(firstSampleCollect)))
                .setLinearHeadingInterpolation(behindBasketPreload.getHeading(), firstSampleCollect.getHeading())
                .build();

        firstSampleScorePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(firstSampleCollect),
                        //new Point(-47.9, 88.1, Point.CARTESIAN),
                        new Point(behindBasketFirstSample)))
                .setConstantHeadingInterpolation(behindBasketFirstSample.getHeading())
                .build();


        /// SECOND SAMPLE

        secondSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindBasketFirstSample), new Point(secondSampleCollect)))
                .setLinearHeadingInterpolation(behindBasketFirstSample.getHeading(), secondSampleCollect.getHeading())
                .build();

        secondSampleScorePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(secondSampleCollect),
                        new Point(behindBasketSecondSample)))
                .setConstantHeadingInterpolation(behindBasketSecondSample.getHeading())
                .build();

        /// THIRD SAMPLE

        thirdSampleCollectPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindBasketSecondSample), new Point(thirdSampleCollect)))
                .setLinearHeadingInterpolation(behindBasketSecondSample.getHeading(), thirdSampleCollect.getHeading())
                .build();

        thirdSampleScorePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(thirdSampleCollect),
                        new Point(behindBasketThirdSample)))
                .setConstantHeadingInterpolation(behindBasketThirdSample.getHeading())
                .build();

        ///  COLLECT FROM SUBMERSIBLE
        ///  FIRST
        firstSubmersibleCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(behindBasketThirdSample),
                        intermediaryExtraPoseForCurves,
                        new Point(firstSubmersibleStdCollect)))
                .setLinearHeadingInterpolation(behindBasketThirdSample.getHeading(), firstSubmersibleStdCollect.getHeading())
                .build();

        firstSubmersibleScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSubmersibleStdCollect),
                        intermediaryExtraPoseForCurves,
                        new Point(firstSubmersibleStdBehindBasket)))
                .setLinearHeadingInterpolation(firstSubmersibleStdCollect.getHeading(), firstSubmersibleStdBehindBasket.getHeading())
                .build();

        /// SECOND
        secondSubmersibleCollectPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSubmersibleStdBehindBasket),
                        intermediaryExtraPoseForCurves,
                        new Point(secondSubmersibleStdCollect)))
                .setLinearHeadingInterpolation(firstSubmersibleStdBehindBasket.getHeading(), secondSubmersibleStdCollect.getHeading())
                .build();

        secondSubmersibleScorePath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondSubmersibleStdCollect),
                        intermediaryExtraPoseForCurves,
                        new Point(secondSubmersibleStdBehindBasket)))
                .setLinearHeadingInterpolation(secondSubmersibleStdCollect.getHeading(), secondSubmersibleStdBehindBasket.getHeading())
                .build();

        /// SECOND TRY AT SUBMESRIBLE

        secondTrySubPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSubmersibleStdCollect),
                        new Point(secondTrySub)))
                .setLinearHeadingInterpolation(firstSubmersibleStdCollect.getHeading(), secondTrySub.getHeading())
                .build();


        /// end autonomy
        signOffPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondSubmersibleStdBehindBasket),
                        intermediaryExtraPoseForCurves,
                        new Point(endingPosition)))
                .setLinearHeadingInterpolation(secondSubmersibleStdBehindBasket.getHeading(), endingPosition.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths
    private final int basketDropTimer = 75;
    public void autonomousPathUpdate() {
        switch (pathState) {
            // PRELOAD
            case 0:
                if(!follower.isBusy()){
                    outtakeBasket();
                    follower.followPath(preloadScorePath,true);


                    setPathState(2);

                }
                break;
            // FIRST SAMPLE
            case 2:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);

                    follower.followPath(firstSampleCollectPath,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    //skipBasket =  executeAutoCollect();
                    // skipBasket = false;
                    executeAutoCollect();

                    //checking after the method finished in case it barely caught it
                    if(currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                        setPathState(4);
                    }
                    else if(currentStateOfSampleInIntake == colorSensorOutty.noSample){
                        setPathState(6);
                    }
                    else{
                        somethingFailed = true;
                        setPathState(6);
                    }

                }
                break;
            case 4:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    waitWhile(300);
                    follower.followPath(firstSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);

                    follower.followPath(secondSampleCollectPath,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    //skipBasket =  executeAutoCollect();
                    // skipBasket = false;
                    executeAutoCollect();

                    //checking after the method finished in case it barely caught it
                    if(currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                        setPathState(8);
                    }
                    else if(currentStateOfSampleInIntake == colorSensorOutty.noSample){
                        setPathState(10);
                    }
                    else{
                        somethingFailed = true;
                        setPathState(10);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    waitWhile(300);
                    follower.followPath(secondSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(10);
                }
                break;


                // THIRD SAMPLE

            case 10:
                if(!follower.isBusy()){
                    follower.followPath(thirdSampleCollectPath,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    //skipBasket =  executeAutoCollect();
                    // skipBasket = false;
                    executeAutoCollect();

                    //checking after the method finished in case it barely caught it
                    if(currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample){
                        setPathState(12);
                    }
                    else if(currentStateOfSampleInIntake == colorSensorOutty.noSample){
                        setPathState(14);
                    }
                    else{
                        somethingFailed = true;
                        setPathState(14);
                    }

                }
                break;
            case 12:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    executeAutoTransfer();
                    //waitWhile(500);
                    follower.followPath(thirdSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(14);
                }
                break;

            /// SUBMERSIBLE COLLECT
            /// FIRST
            case 14:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    autoOuttakeTransfer();
                    follower.followPath(firstSubmersibleCollectPath);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()){
                    executeSubmersibleCollect(true);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors,teamColor,false);
                    if(currentStateOfSampleInIntake == colorSensorOutty.correctSample){
                        setPathState(16);
                    }
                    else{
                        setPathState(100);
                    }
                }
                break;
            case 100:
                if(!follower.isBusy()){
                    follower.followPath(secondTrySubPath);
                    setPathState(101);
                }
                break;
            case 101:
                if(!follower.isBusy()){
                    executeSubmersibleCollect(false);
                    setPathState(16);
                }
                break;

            case 16:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()){
                    follower.followPath(firstSubmersibleScorePath);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(19);
                }
                break;
            /// SECOND
            case 19:
                if(!follower.isBusy()){
                    waitWhile(100); //lets not hang
                    autoOuttakeTransfer();
                    follower.followPath(secondSubmersibleCollectPath);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()){
                    executeSubmersibleCollect(true);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors,teamColor,false);
                    if(currentStateOfSampleInIntake == colorSensorOutty.correctSample){
                        setPathState(21);
                    }
                    else {
                        setPathState(102);
                    }
                }
                break;
            case 102:
                if(!follower.isBusy()){
                    follower.followPath(secondTrySubPath);
                    setPathState(103);
                }
                break;
            case 103:
                if(!follower.isBusy()){
                    executeSubmersibleCollect(false);
                    setPathState(20);
                }
                break;
            case 21:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()){
                    follower.followPath(secondSubmersibleScorePath);
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(24);
                }
                break;
            // sign off
            case 24:
                if(!follower.isBusy()){
                    follower.followPath(signOffPath);
                    waitWhile(50);
                    outtakePark();
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        resetStuff();
        isRobotInAuto = true;
        somethingFailed = false;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstantsForBasket.class, LConstants.class);
        follower = new Follower(hardwareMap,FConstantsForBasket.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        setPathState(0);

        //our init
        intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");

        //declare servos
        intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        isYellowSampleNotGood = true;

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);
        //end of our stuff


        //telemetry stuff
        tel.addData("TeamColor",teamColor);
        tel.update();

        currentTeam = teamColor;
    }


    //for the prearanged samples
    public void executeAutoCollect(){
        autoTimer = System.currentTimeMillis();

        intakeCabinDownCollecting();
        intakeExtended1out4();

        waitWhile(200);
        autoOuttakeTransfer();

        waitWhile(300);
        intakeExtended4out4();

        autoTimer = System.currentTimeMillis();
        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample
                || currentStateOfSampleInIntake == colorSensorOutty.correctSample)
                && autoTimer + 2000 > System.currentTimeMillis()) {
            robotDoStuff();
        }
        intakeCabinTransferPositionWithPower();
        waitWhile(200);
        //if (currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample){
        //    intakeSpinMotorPow = 0;
        //}
        intakeRetracted();
        outtakeClawServoPos = outtakeClawServoExtendedPos;
    }



    public void executeSubmersibleCollect(boolean firstTry) {
        autoTimer = System.currentTimeMillis();
        autoOuttakeTransfer();
        waitWhile(300);
        intakeExtended2out4();
        while(intakeMotor.getCurrentPosition() < 150) {
            robotDoStuff();
        }
        intakeCabinDownCollecting();
        waitWhile(500);
        intakeExtended4out4();
        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample ||
                currentStateOfSampleInIntake == colorSensorOutty.correctSample)
                && autoTimer + 2000 > System.currentTimeMillis()) {
            robotDoStuff();
        }
        NormalizedRGBA colors;
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,teamColor,false);
        if (firstTry){
            if(currentStateOfSampleInIntake==colorSensorOutty.wrongSample){
                intakeExtended2out4();
                intakeSpinMotorPow = 1;
                waitWhile(500);
            } else if(currentStateOfSampleInIntake==colorSensorOutty.correctSample){
                intakeCabinTransferPositionWithPower();
                intakeRetracted();
                outtakeClawServoPos = outtakeClawServoExtendedPos + 20;
            }
        } else {
            if(currentStateOfSampleInIntake==colorSensorOutty.wrongSample){
                intakeSpinMotorPow = 1;
                waitWhile(500);
            }
            intakeCabinTransferPositionWithPower();
            intakeRetracted();
            outtakeClawServoPos = outtakeClawServoExtendedPos + 20;
        }

    }
    public void executeAutoTransfer() {
        autoOuttakeTransfer();
        while(intakeMotor.getCurrentPosition() > 30) {
            robotDoStuff();
        }
        waitWhile(75);
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        waitWhile(200);
        intakeSpinMotorPow = 0;
        outtakeBasket();
    }



    public void robotDoStuff(){
        //risky
        //follower.update();

        //ifs

        //intake stop spinning
        if(shouldStopIntakeCabinSpinningAfterTakig && shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis()){
            intakeSpinMotorPow = -0.8;
            shouldStopIntakeCabinSpinningAfterTakig = false;
            hasSmolOutputed = true;
            hasSmolOutputedTimer = System.currentTimeMillis();
        }
        //and then stop the power stuff
        if(hasSmolOutputed && hasSmolOutputedTimer + 50 <System.currentTimeMillis()){
            intakeCabinTransferPosition();
            if(isInSpecimenState){
                intakeCabinFullInBot();
            }
            hasSmolOutputed = false;
        }

        //*/

        //color stuff
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,teamColor,isYellowSampleNotGood);

        //PID Stuff

        intakeMotorPower = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
        if(currentStateOfSampleInIntake == colorSensorOutty.correctSample) intakeMotorPower *= 1.5;

        outakeMotorPower = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos, outakeLeftMotor.getCurrentPosition());

        //set motor positions
        intakeMotor.setPower(intakeMotorPower);
        outakeRightMotor.setPower(outakeMotorPower);
        outakeLeftMotor.setPower(outakeMotorPower);
        intakeSpinMotor.setPower(intakeSpinMotorPow);
        tel.addData("intakeSpinMotor",intakeSpinMotor.getPower());
        tel.addData("intakeSpinMotorInCode",intakeSpinMotorPow);

        //Set servo Positions
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);

        robotTelemetry();
    }


    public void waitWhile(int timeToWait) {
        long iniTime = System.currentTimeMillis();
        while(iniTime + timeToWait > System.currentTimeMillis()){
            robotDoStuff();
        }
    }


    void robotTelemetry(){
        tel.addData("path state", pathState);
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", follower.getPose().getHeading());
        tel.addData("intakePivotServoPos",intakePivotServoPos);
        tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
        tel.addData("outtakeDirection",outakeMotorPower);
        tel.addData("outtakeCurrenPos",outakeLeftMotor.getCurrentPosition());
        tel.addData("sensor color",currentStateOfSampleInIntake);
        tel.addData("follower is busy",follower.isBusy());
        tel.addData("intake motor", intakeMotor.getCurrentPosition());
        tel.addData("TeamColor",teamColor);
        Drawing.drawDebug(follower);
        tel.update();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        robotDoStuff();


        // Feedback to Driver Hub
        robotTelemetry();
    }



    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //some color code went here
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }




}
