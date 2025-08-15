package pedroPathing.Autos.Linear;

import static pedroPathing.ClassWithStates.ColorCompare;
import static pedroPathing.ClassWithStates.autoOuttakeTransfer;
import static pedroPathing.ClassWithStates.colorList;
import static pedroPathing.ClassWithStates.colorSensorOutty;
import static pedroPathing.ClassWithStates.currentStateOfSampleInIntake;
import static pedroPathing.ClassWithStates.currentTeam;
import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.ClassWithStates.intakeCabinDownCollecting;
import static pedroPathing.ClassWithStates.intakeCabinFullInBot;
import static pedroPathing.ClassWithStates.intakeCabinTransferPosition;
import static pedroPathing.ClassWithStates.intakeCabinTransferPositionWithPower;
import static pedroPathing.ClassWithStates.intakeExtended1out4;
import static pedroPathing.ClassWithStates.intakeExtended2out4;
import static pedroPathing.ClassWithStates.intakeExtended4out4;
import static pedroPathing.ClassWithStates.intakeRetracted;
import static pedroPathing.ClassWithStates.outtakeBasket;
import static pedroPathing.ClassWithStates.outtakePark;
import static pedroPathing.ClassWithStates.outtakeStandByWithoutExtensions;
import static pedroPathing.OrganizedPositionStorage.autoTimer;
import static pedroPathing.OrganizedPositionStorage.hasSmolOutputed;
import static pedroPathing.OrganizedPositionStorage.hasSmolOutputedTimer;
import static pedroPathing.OrganizedPositionStorage.intakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.intakeSpinMotorPow;
import static pedroPathing.OrganizedPositionStorage.intakeTargetPosAdder;
import static pedroPathing.OrganizedPositionStorage.isInSpecimenState;
import static pedroPathing.OrganizedPositionStorage.isRobotInAuto;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoExtendedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoRetractedPos;
import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.resetStuff;
import static pedroPathing.OrganizedPositionStorage.shouldStopIntakeCabinSpinningAfterTakig;
import static pedroPathing.OrganizedPositionStorage.shouldStopIntakeCabinSpinningAfterTakigTimer;
import static pedroPathing.OrganizedPositionStorage.somethingFailed;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import pedroPathing.constants.FConstantsForPinpointForBasket;
import pedroPathing.constants.LConstantsForPinpoint;

//@Config
//@Autonomous(name = "Basket Auto RED ALLIANCE Mateis (nu va exploda)", group = "Examples")
public class AutoForBasketMainNewTuneForPinpoint extends LinearOpMode {
    /// FOARTE IMPORTANT: CULOAREA ALIANTEI!
    private colorList teamColor = colorList.red;

    public AutoForBasketMainNewTuneForPinpoint(colorList color){
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
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); //start

    // place in basket
    public final double basketY = -6;//45097194883;
    public final double basketX = 44;//71182947835;

    double basketHeading = Math.toRadians(53.65523062885758);
    private final Pose behindBasketPreload = new Pose(basketX, basketY, (basketHeading));
    private final Pose behindBasketFirstSample = new Pose(basketX, basketY, (basketHeading));
    private final Pose behindBasketSecondSample = new Pose(basketX, basketY, (basketHeading));
    private final Pose behindBasketThirdSample = new Pose(basketX, basketY-0.7 , (basketHeading));

    // collect first 3 from floor
    private final Pose firstSampleCollect = new Pose(43.85852723609744, -7.929813054602916, Math.toRadians(74.34624028537361));
    private final Pose secondSampleCollect = new Pose(44.9, -7.604137630913201, Math.toRadians(91.70738587244128));
    private final Pose thirdSampleCollect = new Pose(38.766777459092026, -8.76637015755721, Math.toRadians(126.5));

    // collect from submersible

    // first
    private final Pose firstSubmersibleStdCollect = new Pose(8, -53.95808182363435, Math.toRadians(1.343121268676688));
    private final Pose firstSubmersibleStdBehindBasket = new Pose(basketX+2, basketY, basketHeading-Math.toRadians(15));

    // second
    private final Pose secondSubmersibleStdCollect = new Pose(8, -59.30385799858514, Math.toRadians(1.996540493692655));
    private final Pose secondSubmersibleStdBehindBasket = new Pose(basketX+2, basketY, basketHeading-Math.toRadians(15));
    // sign off
    private final Pose endingPosition = new Pose(0, 0, Math.toRadians(0));

    private final Pose secondTrySub = new Pose(6.043441652312993 + 1, -49.70520500123031, 6.203226558611047);

    private final Point intermediaryExtraPoseForCurves = new Point( 52 ,-40);
    /*
    heading: 4.0289046655823935
intakerotate: 40.0
path state: 0
x: -52.47229868971458
y: 83.09612604576769
slides pos on descent -223
     */

    private boolean skipBasket = false;
    private boolean failedFirstSample = false;
    private boolean failedSecondSample = false;
    private boolean failedThirdSample = false;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    //private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    //private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;

    private PathChain firstSampleCollectPath, preloadScorePath, firstSampleScorePath, secondSampleCollectPath, secondSampleScorePath, thirdSampleCollectPath, thirdSampleScorePath
            , firstSubmersibleCollectPath, firstSubmersibleScorePath,
            secondSubmersibleCollectPath, secondSubmersibleScorePath
            ,secondTrySubPath, signOffPath, pickUpSecondSampleAfterMissedFirst, pickUpThirdSampleAfterMissedSecond, pickUpFromSubmersibleAfterMissedThird;

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

        /// missed stuff
        pickUpSecondSampleAfterMissedFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSampleCollect), new Point(secondSampleCollect)))
                .setLinearHeadingInterpolation(firstSampleCollect.getHeading(), secondSampleCollect.getHeading())
                .build();

        pickUpThirdSampleAfterMissedSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSampleCollect), new Point(thirdSampleCollect)))
                .setLinearHeadingInterpolation(secondSampleCollect.getHeading(), thirdSampleCollect.getHeading())
                .build();

        pickUpFromSubmersibleAfterMissedThird = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdSampleCollect),
                        intermediaryExtraPoseForCurves,
                        new Point(firstSubmersibleStdCollect)))
                .setLinearHeadingInterpolation(thirdSampleCollect.getHeading(), firstSubmersibleStdCollect.getHeading())
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
    private final int basketDropTimer = 50;
    public void autonomousPathUpdate() {
        switch (pathState) {
            // PRELOAD
            case 0:
                if (!follower.isBusy()) {
                    outtakeBasket();
                    follower.followPath(preloadScorePath, true);


                    setPathState(2);

                }
                break;
            // FIRST SAMPLE
            case 2:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    intakeCabinDownCollecting();

                    follower.followPath(firstSampleCollectPath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    executeAutoCollectFast();

                    //checking after the method finished in case it barely caught it
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample) {
                        setPathState(4);
                    } else if (currentStateOfSampleInIntake == colorSensorOutty.noSample) {
                        setPathState(6);
                        failedFirstSample = true;
                    } else {
                        failedFirstSample = true;
                        somethingFailed = true;
                        setPathState(6);
                    }

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    executeAutoTransfer();
                    waitWhile(300);
                    follower.followPath(firstSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    intakeCabinDownCollecting();

                    if(!failedFirstSample) follower.followPath(secondSampleCollectPath, true);
                    else follower.followPath(pickUpSecondSampleAfterMissedFirst, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    executeAutoCollectFast();

                    //checking after the method finished in case it barely caught it
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample) {
                        setPathState(8);
                    } else if (currentStateOfSampleInIntake == colorSensorOutty.noSample) {
                        failedSecondSample = true;
                        setPathState(10);
                    } else {
                        failedSecondSample = true;
                        somethingFailed = true;
                        setPathState(10);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    executeAutoTransfer();
                    waitWhile(300);
                    follower.followPath(secondSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    intakeCabinDownCollecting();

                    setPathState(10);
                }
                break;


            // THIRD SAMPLE

            case 10:
                if (!follower.isBusy()) {
                    if(!failedSecondSample) follower.followPath(thirdSampleCollectPath, true);
                    else follower.followPath(pickUpThirdSampleAfterMissedSecond, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    executeAutoCollectFast();

                    //checking after the method finished in case it barely caught it
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample || currentStateOfSampleInIntake == colorSensorOutty.wrongSample) {
                        setPathState(12);
                    } else if (currentStateOfSampleInIntake == colorSensorOutty.noSample) {
                        failedThirdSample = true;
                        setPathState(14);
                    } else {
                        failedThirdSample = true;
                        somethingFailed = true;
                        setPathState(14);
                    }

                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    waitWhile(100); //lets not hang
                    executeAutoTransfer();
                    //waitWhile(500);
                    follower.followPath(thirdSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(14);
                }
                break;

            /// SUBMERSIBLE COLLECT
            /// FIRST
            case 14:
                if (!follower.isBusy()) {
                    waitWhile(100); //lets not hang
                    autoOuttakeTransfer();

                    if(!failedThirdSample ) follower.followPath(firstSubmersibleCollectPath);
                    else follower.followPath(pickUpFromSubmersibleAfterMissedThird);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    executeSubmersibleCollect(true);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors, teamColor, false);
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample) {
                        setPathState(16);
                    } else {
                        setPathState(100);
                    }
                }
                break;
            case 100:
                if (!follower.isBusy()) {
                    follower.followPath(secondTrySubPath);
                    executeSubmersibleCollect(false);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors, teamColor, false);
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample) {
                        setPathState(16);
                    } else {
                        setPathState(100);
                    }
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    executeAutoTransfer();
                    outtakeStandByWithoutExtensions();
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(firstSubmersibleScorePath);
                    waitWhile(300);
                    outtakeBasket();
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(19);
                }
                break;
            /// SECOND
            case 19:
                if (!follower.isBusy()) {
                    waitWhile(100); //lets not hang
                    autoOuttakeTransfer();
                    follower.followPath(secondSubmersibleCollectPath);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    executeSubmersibleCollect(true);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors, teamColor, false);
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample) {
                        setPathState(21);
                    } else {
                        setPathState(102);
                    }
                }
                break;
            case 102:
                if (!follower.isBusy()) {
                    follower.followPath(secondTrySubPath);
                    executeSubmersibleCollect(false);
                    NormalizedRGBA colors;
                    colors = colorSensor.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    currentStateOfSampleInIntake = ColorCompare(colors, teamColor, false);
                    if (currentStateOfSampleInIntake == colorSensorOutty.correctSample) {
                        setPathState(21);
                    } else {
                        setPathState(102);
                    }
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    executeAutoTransfer();
                    outtakeStandByWithoutExtensions();
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(secondSubmersibleScorePath);
                    waitWhile(300);
                    outtakeBasket();
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(basketDropTimer);
                    setPathState(24);
                }
                break;
            // sign off
            case 24:
                if (!follower.isBusy()) {
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
                && autoTimer + 2000 > System.currentTimeMillis()
                && !isStopRequested()) {
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


    public void executeAutoCollectFast(){
        autoTimer = System.currentTimeMillis();

        intakeCabinDownCollecting();
        autoOuttakeTransfer();
        intakeExtended4out4();

        autoTimer = System.currentTimeMillis();
        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample
                || currentStateOfSampleInIntake == colorSensorOutty.correctSample)
                && autoTimer + 2000 > System.currentTimeMillis()
                && !isStopRequested()) {
            robotDoStuff();
        }
        intakeCabinTransferPositionWithPower();
        intakeRetracted();
        outtakeClawServoPos = outtakeClawServoExtendedPos;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        resetStuff();
        isRobotInAuto = true;
        somethingFailed = false;

        failedFirstSample = false;
        failedSecondSample = false;
        failedThirdSample = false;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstantsForPinpointForBasket.class, LConstantsForPinpoint.class);
        follower = new Follower(hardwareMap,FConstantsForPinpointForBasket.class,LConstantsForPinpoint.class);
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

        waitForStart();

        if (isStopRequested()){
            return;
        }
        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            robotDoStuff();


            // Feedback to Driver Hub
            robotTelemetry();
        }
    }

    public void executeSubmersibleCollect(boolean firstTry) {
        autoTimer = System.currentTimeMillis();
        autoOuttakeTransfer();
        waitWhile(300);
        intakeExtended1out4();
        while(intakeMotor.getCurrentPosition() < 100) {
            robotDoStuff();
        }
        intakeCabinDownCollecting();
        waitWhile(500);
        intakeExtended4out4();
        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample ||
                currentStateOfSampleInIntake == colorSensorOutty.correctSample)
                && autoTimer + 2000 > System.currentTimeMillis()
                && !isStopRequested()
        ) {
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
                intakeSpinMotorPow = 0.8;
                waitWhile(35);
                intakeCabinTransferPositionWithPower();
                intakeRetracted();
                outtakeClawServoPos = outtakeClawServoExtendedPos + 20;
            }
        } else {
            if(currentStateOfSampleInIntake==colorSensorOutty.wrongSample){
                intakeSpinMotorPow = 1;
                waitWhile(500);
            }
            intakeSpinMotorPow = 0.8;
            waitWhile(35);
            intakeCabinTransferPositionWithPower();
            intakeRetracted();
            outtakeClawServoPos = outtakeClawServoExtendedPos + 20;
        }

    }
    public void executeAutoTransfer() {
        autoOuttakeTransfer();
        while(intakeMotor.getCurrentPosition() > 30
                && !isStopRequested()) {
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
        if(isStopRequested()) requestOpModeStop();
        //ifs

        //intake stop spinning
        if(shouldStopIntakeCabinSpinningAfterTakig && shouldStopIntakeCabinSpinningAfterTakigTimer + 500 < System.currentTimeMillis()
                && !isStopRequested()){
            intakeSpinMotorPow = -0.8;
            shouldStopIntakeCabinSpinningAfterTakig = false;
            hasSmolOutputed = true;
            hasSmolOutputedTimer = System.currentTimeMillis();
        }
        //and then stop the power stuff
        if(hasSmolOutputed && hasSmolOutputedTimer + 50 <System.currentTimeMillis()
                && !isStopRequested()){
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
        while(iniTime + timeToWait > System.currentTimeMillis() && !isStopRequested()){
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

}
