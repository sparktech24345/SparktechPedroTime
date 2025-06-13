package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Basket Auto Vision", group = "Examples")
public class BasketTestAutoVision extends OpMode {
    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;


    /// SAMPLE DETECTION VALUES
    private final ColorRange FTC_YELLOW = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 220, 220,   0),  //yellow
            new Scalar(255, 255, 255)
    );
    private final ColorRange FTC_RED = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 215, 0,  0),  //red
            new Scalar(255, 219, 180)
    );
    private final ColorRange FTC_BLUE = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 16,   0, 215), //blue
            new Scalar(255, 127, 255)
    );
    /// NOT SETTINGS
    private ColorBlobLocatorProcessor colorLocatorBlue;
    private ColorBlobLocatorProcessor colorLocatorRed;
    private ColorBlobLocatorProcessor colorLocatorYellow;

    private VisionPortal portal;
    /// END OF SAMPLE DETECTION VALUES

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
    private final Pose parkingPose=new Pose(-55,70 - 0.5,Math.toRadians(90)); //parking


    double intakeMotorPower=0;
    double outakeMotorPower=0;

    // BASKET
    private final Pose startPose = new Pose(-10, 70, Math.toRadians(180)); //start

    // place in basket
    private final float xOffsetBasket = 1.25f;
    private final float yOffsetBasket = 1.5f;
    private final Pose behindBasketPreload = new Pose(-44.3, 80, Math.toRadians(225));
    private final Pose behindBasketFirstSample = new Pose(-48.5 - 2 + xOffsetBasket, 81.5 +1.5 + yOffsetBasket, Math.toRadians(225));
    private final Pose behindBasketSecondSample = new Pose(-49.5 - 4.5 + 3 + xOffsetBasket, 82.5 + 2 + yOffsetBasket, Math.toRadians(225));
    private final Pose behindBasketThirdSample = new Pose(-49 - 0.5 + 1 + xOffsetBasket, 82 + 2 + yOffsetBasket, Math.toRadians(225));

    // collect first 3 from floor
    private final Pose firstSampleCollect = new Pose( -49.25 - 2.5, 91 + 2, Math.toRadians(287));
    private final Pose secondSampleCollect = new Pose(-49.25 - 1, 89 + 2, Math.toRadians(270));
    private final Pose thirdSampleCollect = new Pose( -44.3 - 4.5 + 5.25, 86.20 + 2, Math.toRadians(270));

    /// SUBMERSIBLE
    /// INTERMEDIARY POSES (FOR EXECUTING DETECTION)

    private final Pose firstSubmersibleCollectIntermediary = new Pose();
    private final Pose secondSubmersibleCollectIntermediary = new Pose();

    /// INTERPOLATION VARIABLES
    private double startOfSubmersible = 50;
    private double endOfSubmersible = 50;

    // first
    private Pose firstSubmersibleStdCollect = new Pose();
    private final Pose firstSubmersibleStdBehindBasket = new Pose(-49 - 1.8 + xOffsetBasket, 82 + 2.7 + yOffsetBasket, Math.toRadians(225));

    // second
    private Pose secondSubmersibleStdCollect = new Pose();
    private final Pose secondSubmersibleStdBehindBasket = new Pose(-49 - 2.5 + xOffsetBasket, 82 + 2 + yOffsetBasket, Math.toRadians(225));

    // sign off
    private final Pose endingPosition = new Pose( -49.25 - 2.5, 91 + 2, Math.toRadians(287));

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
            , firstSubmersibleIntermediaryPath, firstSubmersibleCollectPath, firstSubmersibleScorePath,
            secondSubmersibleIntermediaryPath, secondSubmersibleCollectPath, secondSubmersibleScorePath
            , signOffPath;

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
        firstSubmersibleIntermediaryPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(behindBasketThirdSample),
                        new Point(-56, 102),
                        new Point(firstSubmersibleStdCollect)))
                .setLinearHeadingInterpolation(behindBasketThirdSample.getHeading(), firstSubmersibleStdCollect.getHeading())
                .build();

        firstSubmersibleScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSubmersibleStdCollect), new Point(firstSubmersibleStdBehindBasket)))
                .setLinearHeadingInterpolation(firstSubmersibleStdCollect.getHeading(), firstSubmersibleStdBehindBasket.getHeading())
                .build();

        /// SECOND
        secondSubmersibleIntermediaryPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSubmersibleStdBehindBasket),
                        new Point(-56, 102),
                        new Point(secondSubmersibleStdCollect)))
                .setLinearHeadingInterpolation(firstSubmersibleStdBehindBasket.getHeading(), secondSubmersibleStdCollect.getHeading())
                .build();

        secondSubmersibleScorePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSubmersibleStdCollect), new Point(secondSubmersibleStdBehindBasket)))
                .setLinearHeadingInterpolation(secondSubmersibleStdCollect.getHeading(), secondSubmersibleStdBehindBasket.getHeading())
                .build();

        /// end autonomy
        signOffPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSubmersibleStdBehindBasket), new Point(endingPosition)))
                .setLinearHeadingInterpolation(secondSubmersibleStdBehindBasket.getHeading(), endingPosition.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths
    private final int slideExtensionTimer = 300;
    private final int basketDropTimer = 75;
    public void autonomousPathUpdate() {


        switch (pathState) {
            // PRELOAD
            case 0:
                if(!follower.isBusy()){
                    outtakeBasket();
                    follower.followPath(preloadScorePath,true);


                    setPathState(1);

                }
                break;
            case 1:
                if(!follower.isBusy()){
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(2);

                }
                break;

            // FIRST SAMPLE
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(firstSampleCollectPath,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    skipBasket =  executeAutoCollect();
                    if(!skipBasket){
                        setPathState(4);
                    } else {
                        setPathState(6);
                    }

                }
                break;
            case 4:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    //waitWhile(500);
                    follower.followPath(firstSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(6);
                }
                break;

            // SECOND SAMPLE

            case 6:
                if(!follower.isBusy()){
                    follower.followPath(secondSampleCollectPath,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    skipBasket =  executeAutoCollect();
                    if(!skipBasket){
                        setPathState(8);
                    } else {
                        setPathState(10);
                    }

                }
                break;
            case 8:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    //waitWhile(500);
                    follower.followPath(secondSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    waitWhile(basketDropTimer);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
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
                    skipBasket =  executeAutoCollect();
                    if(!skipBasket){
                        setPathState(12);
                    } else {
                        setPathState(14);
                    }

                }
                break;
            case 12:
                if(!follower.isBusy()){
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
                    setPathState(14);
                }
                break;

            /// SUBMERSIBLE COLLECT
            /// FIRST
            case 14:
                if(!follower.isBusy()){
                    autoOuttakeTransfer();
                    follower.followPath(firstSubmersibleIntermediaryPath);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()){
                    executeSubmersibleCollect();
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
                    setPathState(19);
                }
                break;
            /// SECOND
            case 19:
                if(!follower.isBusy()){
                    autoOuttakeTransfer();
                    follower.followPath(secondSubmersibleIntermediaryPath);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()){
                    executeSubmersibleCollect();
                    setPathState(21);
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
                    setPathState(24);
                }
                break;
            // sign off
            case 24:
                if(!follower.isBusy()){
                    follower.followPath(signOffPath);
                    waitWhile(50);
                    autoOuttakeTransfer();
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

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
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

        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)// Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setShowStatsOverlay(false)
                .build();


    }

    public boolean executeAutoCollect(){
        boolean collectTimedOut = true;
        autoTimer = System.currentTimeMillis();
        intakeCabinDownCollecting();
        waitWhile(500);
        autoOuttakeTransfer();
        waitWhile(300);
        intakeExtended4out4();
        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample)
                && autoTimer + 2000 > System.currentTimeMillis()) {
            robotDoStuff();
        }
        waitWhile(200);
        if (currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample){
            collectTimedOut = false;
            intakeSpinMotorPow = 0;
        }
        intakeRetracted();
        intakeCabinTransferPositionWithPower();
        outtakeClawServoPos = outtakeClawServoExtendedPos;
        return collectTimedOut;
    }

    public void executeSubmersibleCollect() {
        autoTimer = System.currentTimeMillis();
        autoOuttakeTransfer();
        waitWhile(300);
        intakeExtended3out4();

        while(intakeMotor.getCurrentPosition() < 150) {
            robotDoStuff();
        }

        intakeCabinDownCollecting();

        while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample)) {
            robotDoStuff();
        }
        intakeCabinTransferPositionWithPower();
        intakeRetracted();
        outtakeClawServoPos = outtakeClawServoExtendedPos + 50;
    }

    public void executeAutoTransfer() {
        autoOuttakeTransfer();
        while(intakeMotor.getCurrentPosition() > 50) {
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
        follower.update();



        /*
        //ifs
        if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
            needsToExtraExtend = false;
            outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
        }

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

        */

        //color stuff
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);

        //PID Stuff
        intakeMotorPower = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
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
        Drawing.drawDebug(follower);
        FtcDashboard.getInstance().startCameraStream(portal, 30);
        dashboardTelemetry.update();
        tel.update();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        ColorBlobLocatorProcessor.Blob selectedBlob = selectFromDetection();

        // These loop the movements of the robot
        follower.update();

        autonomousPathUpdate();

        robotDoStuff();


        // Feedback to Driver Hub
        robotTelemetry();
    }

    public ColorBlobLocatorProcessor.Blob selectFromDetection(){
        ColorBlobLocatorProcessor.Blob selectedBlob = null;

        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(colorLocatorRed.getBlobs());
        blobs.addAll(colorLocatorBlue.getBlobs());
        blobs.addAll(colorLocatorYellow.getBlobs());
        AtomicInteger i = new AtomicInteger(1);
        colorLocatorRed.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {
                //telemetry.addData("Red Blob " + i + " Size Y", blob.getBoxFit().size.height);
                //telemetry.addData("Red Blob " + i + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Red Blob " + i + " Angle", blob.getBoxFit().angle);
                i.getAndIncrement();
            }
        });
        AtomicInteger j = new AtomicInteger(1);
        colorLocatorYellow.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {

                //telemetry.addData();
                //telemetry.addData("Yellow Blob " + j + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Yellow Blob " + j + " Angle", blob.getBoxFit().angle);
                j.getAndIncrement();
            }
        });
        AtomicInteger k = new AtomicInteger(1);
        colorLocatorBlue.getBlobs().forEach( blob -> {
            if(blob.getBoxFit() != null) {
                //telemetry.addData("Blue Blob " + k + " Size Y", blob.getBoxFit().size.height);
                //telemetry.addData("Blue Blob " + k + " Size X", blob.getBoxFit().size.width);
                //telemetry.addData("Blue Blob " + k + " Angle", blob.getBoxFit().angle);
                k.getAndIncrement();
            }
        });


        // Variable to hold the blob with the maximum y coordinate
        int minDistance = Integer.MAX_VALUE; // Initialize maxY to the smallest possible integer

        int xCenter = 320;
        int yCenter = 480;

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            String color = "Cat";
            if (colorLocatorRed.getBlobs().contains(b))
                color = "Red";
            else if (colorLocatorBlue.getBlobs().contains(b))
                color = "Blue";
            else if (colorLocatorYellow.getBlobs().contains(b))
                color = "Yellow";
            else
                continue;

            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

            dashboardTelemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

            int xDistance = (int) boxFit.center.x - xCenter;
            int yDistance = (int) boxFit.center.y - yCenter;
            int distance = xDistance*xDistance + yDistance*yDistance;

            // Check if the current blob has a higher y coordinate than the current maxY
            if (distance < minDistance) {
                minDistance = distance; // Update maxY
                selectedBlob = b; // Update selectedBlob to the current blob
            }
        }

        return selectedBlob;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
            currentTeam = colorList.blue;
        if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
            currentTeam = colorList.red;


        robotTelemetry();
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
