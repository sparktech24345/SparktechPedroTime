package pedroPathing.Autos.Linear;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.PIDStorageAndUse.ControlMotor;
import pedroPathing.PIDStorageAndUse.NewPidsController;
import pedroPathing.constants.FConstantsForPinpoint;
import pedroPathing.constants.LConstantsForPinpoint;

@Config
@Autonomous(name = "5 Spec + 1 Sample the new way", group = "Examples")
public class AutoOf5SpecAnd1TheNewWay extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, intakeGoDownTimer,intakeShouldExtendTimer;
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
    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); //start
    //scoring bar positions
    private final float scoringBarX = -31f;
    private final float scoringBarY = -1f;
    private final Pose scoringBarPosePreloadSpecimen = new Pose(scoringBarX, scoringBarY+0.8, 0); //start
    private final Pose scoringBarPoseFirstSpecimen = new Pose(scoringBarX, scoringBarY+1, 0); //start
    private final Pose scoringBarPoseSecondSpecimen = new Pose(scoringBarX, scoringBarY+1.1, 0); //start
    private final Pose scoringBarPoseThirdSpecimen = new Pose(scoringBarX, scoringBarY+1.3, 0); //start
    private final Pose scoringBarPoseFourthSpecimen = new Pose(scoringBarX, scoringBarY+1.5, 0); //start
    //private final Pose scoringBarPoseFifthSpecimen = new Pose(-5.4, 44 + globalSpecimenYOffset, Math.toRadians(90)); //start

    private final float wallPickUpX = 2.5f;
    private final float wallPickUpY = 30f;

    //specimen pick up positions
    private final Pose firstSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, 0); //start
    private final Pose secondSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, 0); //start
    private final Pose thirdSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, 0); //start
    private final Pose fourthSpecimenPickUpPose = new Pose(wallPickUpX, wallPickUpY, 0); //start

    // ----------------------------------------------- SAMPLE POSES ----------------------------------------------- \\

    private final Pose firstSamplePickUpPos = new Pose(-21.88, 27, Math.toRadians(318));
    private final Pose firstSampleDepositPos = new Pose(-18.160, 34.494, Math.toRadians(239.527));
    private final Pose secondSamplePickUpPos = new Pose(-19.1239, 39.1783, Math.toRadians(322.5674));
    private final Pose secondSampleDepositPos = new Pose(-17.5389, 40.796, Math.toRadians(224.0242));
    private final Pose thirdSamplePickUpPos = new Pose(-20.31, 46, Math.toRadians(317));
    private final Pose thirdSampleDepositPos = new Pose(-17.971, 44.790, Math.toRadians(230.693));
    private final Pose intermediatePos = new Pose(-18, 30, Math.toRadians(0));

    // --------- + 1 ------------- \\
    private final Pose basketPickUp = new Pose(-6, 13, 4.672053639088766);
    private final Pose basketScore = new Pose(-7, -59, 5.327179614697592);

    //PARK
    private final Pose parkingPose=new Pose(-6.625476146307517, 8.210622832531065, 4.672053639088766); //parking
    double intakeMotorPower=0;
    double outakeMotorPower=0;
    boolean shouldIntakeGoDown = false;
    boolean shouldIntakeExtendForLateColect = false;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking,scoreInBasket;
    private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample,goToPickUpForBasket,depositFirstSample,depositSecondSample,depositThirdSample,goToIntermidiate;


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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPosePreloadSpecimen)));
        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPosePreloadSpecimen.getHeading());

        goToPickUpFirstSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPosePreloadSpecimen), new Point(firstSamplePickUpPos)))
                .setLinearHeadingInterpolation(scoringBarPosePreloadSpecimen.getHeading(), firstSamplePickUpPos.getHeading())
                .build();

        depositFirstSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSamplePickUpPos), new Point(firstSampleDepositPos)))
                .setLinearHeadingInterpolation(firstSamplePickUpPos.getHeading(), firstSampleDepositPos.getHeading())
                .build();

        goToPickUpSecondSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSampleDepositPos), new Point(secondSamplePickUpPos)))
                .setLinearHeadingInterpolation(firstSampleDepositPos.getHeading(), secondSamplePickUpPos.getHeading())
                .build();

        depositSecondSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePickUpPos), new Point(secondSampleDepositPos)))
                .setLinearHeadingInterpolation(secondSamplePickUpPos.getHeading(), secondSampleDepositPos.getHeading())
                .build();

        goToPickUpThirdSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSampleDepositPos), new Point(thirdSamplePickUpPos)))
                .setLinearHeadingInterpolation(secondSampleDepositPos.getHeading(), thirdSamplePickUpPos.getHeading())
                .build();

        depositThirdSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSamplePickUpPos), new Point(thirdSampleDepositPos)))
                .setLinearHeadingInterpolation(thirdSamplePickUpPos.getHeading(), thirdSampleDepositPos.getHeading())
                .build();

        goToIntermidiate =follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSampleDepositPos), new Point(intermediatePos)))
                .setLinearHeadingInterpolation(thirdSampleDepositPos.getHeading(), intermediatePos.getHeading())
                .build();


        //first spec

        pickUpFirst = new Path(new BezierLine(new Point(intermediatePos), new Point(firstSpecimenPickUpPose)));
        pickUpFirst.setLinearHeadingInterpolation(intermediatePos.getHeading(), firstSpecimenPickUpPose.getHeading());

        scoreFirst = new Path(new BezierLine(new Point(firstSpecimenPickUpPose), new Point(scoringBarPoseFirstSpecimen)));
        scoreFirst.setLinearHeadingInterpolation(firstSpecimenPickUpPose.getHeading(), scoringBarPoseFirstSpecimen.getHeading());

        //second spec

        pickUpSecond = new Path(new BezierLine(new Point(scoringBarPoseFirstSpecimen), new Point(secondSpecimenPickUpPose)));
        pickUpSecond.setLinearHeadingInterpolation(scoringBarPoseFirstSpecimen.getHeading(), secondSpecimenPickUpPose.getHeading());

        scoreSecond = new Path(new BezierLine(new Point(secondSpecimenPickUpPose), new Point(scoringBarPoseSecondSpecimen)));
        scoreSecond.setLinearHeadingInterpolation(secondSpecimenPickUpPose.getHeading(), scoringBarPoseSecondSpecimen.getHeading());

        //third spec

        pickUpThird = new Path(new BezierLine(new Point(scoringBarPoseSecondSpecimen), new Point(thirdSpecimenPickUpPose)));
        pickUpThird.setLinearHeadingInterpolation(scoringBarPoseSecondSpecimen.getHeading(), thirdSpecimenPickUpPose.getHeading());

        scoreThird = new Path(new BezierLine(new Point(thirdSpecimenPickUpPose), new Point(scoringBarPoseThirdSpecimen)));
        scoreThird.setLinearHeadingInterpolation(thirdSpecimenPickUpPose.getHeading(), scoringBarPoseThirdSpecimen.getHeading());


        //fourth spec

        pickUpFourth = new Path(new BezierLine(new Point(scoringBarPoseThirdSpecimen), new Point(fourthSpecimenPickUpPose)));
        pickUpFourth.setLinearHeadingInterpolation(scoringBarPoseThirdSpecimen.getHeading(), fourthSpecimenPickUpPose.getHeading());

        scoreFourth = new Path(new BezierLine(new Point(fourthSpecimenPickUpPose), new Point(scoringBarPoseFourthSpecimen)));
        scoreFourth.setLinearHeadingInterpolation(fourthSpecimenPickUpPose.getHeading(), scoringBarPoseFourthSpecimen.getHeading());

        ///  + 1
        goToPickUpForBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPoseFourthSpecimen), new Point(basketPickUp)))
                .setLinearHeadingInterpolation(scoringBarPoseFourthSpecimen.getHeading(), basketPickUp.getHeading())
                .build();

        scoreInBasket = new Path(new BezierLine(new Point(basketPickUp), new Point(basketScore)));
        scoreInBasket.setLinearHeadingInterpolation(basketPickUp.getHeading(), basketScore.getHeading());


//        scoreInBasket = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(basketPickUp), new Point(basketScore)))
//                .setLinearHeadingInterpolation(basketPickUp.getHeading(), basketScore.getHeading())
//                .build();

        ///  NO PARKING
        //parking
        parking = new Path(new BezierLine(new Point(basketScore), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(basketScore.getHeading(), parkingPose.getHeading());

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    //init
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(startPath,true);
                    setPathState(2);
                }
                break;

                //score preload
            case 2:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(100);
                    autoTimer = System.currentTimeMillis();
                    intakeGoDownTimer = new Timer();
                    intakeGoDownTimer.resetTimer();
                    shouldIntakeGoDown = true;
                    follower.followPath(goToPickUpFirstSample,true);
                    setPathState(3);
                }
                break;

                //pick up first sample
            case 3:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    intakeExtended3out4();
                    waitWhile(200);
                    intakeExtended4out4();
                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested())  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();

                    //after collection
                    intakeExtended1out4();
                    intakeCabinALittleBitUpStandStill();
                    follower.followPath(depositFirstSample,true);
                    setPathState(4);
                }
                break;

                //deposit first sample
            case 4:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeExtended4out4();
                    waitWhile(50);
                    intakeCabinALittleBitUpOutputting();
                    autoTimer = System.currentTimeMillis();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested()) robotDoStuff();

                    //wait for proper sample exit
                    waitWhile(150);

                    intakeCabinDownCollecting();
                    intakeExtended1out4();

                    follower.followPath(goToPickUpSecondSample,true);
                    setPathState(5);
                }
                break;

                //pickup second sample
            case 5:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    intakeExtended3out4();
                    waitWhile(200);
                    intakeExtended4out4();
                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested())  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();

                    //after collection
                    intakeExtended1out4();
                    intakeCabinALittleBitUpStandStill();
                    follower.followPath(depositSecondSample,true);
                    setPathState(6);
                }
                break;

            //deposit second sample
            case 6:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeExtended4out4();
                    waitWhile(50);
                    intakeCabinALittleBitUpOutputting();
                    autoTimer = System.currentTimeMillis();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested()) robotDoStuff();

                    //wait for proper sample exit
                    waitWhile(150);

                    intakeCabinDownCollecting();
                    intakeExtended1out4();

                    follower.followPath(goToPickUpThirdSample,true);
                    setPathState(7);
                }
                break;

                //pick up third sample
            case 7:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    intakeExtended3out4();
                    waitWhile(200);
                    intakeExtended4out4();
                    autoTimer = System.currentTimeMillis();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested())  && autoTimer + 2000 > System.currentTimeMillis()) robotDoStuff();

                    //after collection
                    intakeExtended1out4();
                    intakeCabinALittleBitUpStandStill();
                    follower.followPath(depositThirdSample,true);
                    setPathState(8);
                }
                break;

            //third sample deposit and first spec prep
            case 8:
                if(!follower.isBusy()) {

                    //deposit 3rd spec
                    autoTimer = System.currentTimeMillis();
                    intakeExtended4out4();
                    waitWhile(50);
                    intakeCabinALittleBitUpOutputting();
                    autoTimer = System.currentTimeMillis();
                    while(currentStateOfSampleInIntake == colorSensorOutty.correctSample && !isStopRequested()) robotDoStuff();



                    //prep for pick up
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoOuttakeWallPickUpNew();
                    intakeRetracted();
                    intakeCabinFullInBot();
                    autoTimer = System.currentTimeMillis();


                    follower.followPath(goToIntermidiate,false);
                    setPathState(9);
                }
                break;


            //intermediate pos
            case 9:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpFirst,true);
                    setPathState(106);
                }
                break;
            case 106:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreFirst,true);
                    setPathState(107);
                }
                break;

            //second spec
            case 107:
                if(!follower.isBusy()) {
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpSecond,true);
                    setPathState(108);
                }
                break;
            case 108:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreSecond,true);
                    setPathState(109);
                }
                break;

            //third spec
            case 109:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpThird,true);
                    setPathState(110);
                }
                break;
            case 110:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreThird,true);
                    setPathState(111);
                }
                break;

            //fourth spec
            case 111:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoOuttakeWallPickUpNew();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpFourth,true);
                    setPathState(112);
                }
                break;
            case 112:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreFourth,true);
                    setPathState(113);
                }
                break;


            case 113:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoTimer = System.currentTimeMillis();

                    intakeGoDownTimer.resetTimer();
                    shouldIntakeGoDown = true;


                    intakeShouldExtendTimer = new Timer();
                    intakeShouldExtendTimer.resetTimer();
                    shouldIntakeExtendForLateColect = true;

                    follower.followPath(goToPickUpForBasket,true);
                    setPathState(114);
                }
                break;
            ///  COLLECTING FROM OBSERVATION ZONE
            case 114:
                if(!follower.isBusy()) {
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
                    waitWhile(100);
                    //if (currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample){
                    //    intakeSpinMotorPow = 0;
                    //}
                    intakeRetracted();
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(115);
                }
                break;
            /// TRANSFER
            case 115:
                if(!follower.isBusy()) {
                    shouldDoAutoSpecInTeleopBeggining = true;
                    follower.followPath(scoreInBasket);
                    waitWhile(0);
                    autoOuttakeTransfer();
                    while(intakeMotor.getCurrentPosition() > 30 && !isStopRequested()) {
                        robotDoStuff();
                    }
                    waitWhile(75);
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(200);
                    intakeSpinMotorPow = 0;
                    outtakeBasket();
                    waitWhile(1500);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(117);
                }
                break;
            case 117:
                if(!follower.isBusy()){
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(100);
                    follower.followPath(parking);
                    waitWhile(500);
                    intakeExtended4out4();
                    intakeCabinTransferPosition();
                    autoOuttakeTransfer();
                    setPathState(118);
                }
                break;
            case 118:
                if(!follower.isBusy()){
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

    @Override
    public void runOpMode() throws InterruptedException {


        //init


        resetStuff();
        isRobotInAuto = true;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        follower = new Follower(hardwareMap, FConstantsForPinpoint.class,LConstantsForPinpoint.class);
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


        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);
        //end of our stuff
        //end of init

        //boolean shouldDoStart = true;

        waitForStart();

        if (isStopRequested()){
            return;
        }


        opmodeTimer.resetTimer();
        setPathState(0);


        while(opModeIsActive()){
            //one time thing
            //if(shouldDoStart){
            //    shouldDoStart = false;
            //}

            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            robotDoStuff();


            // Feedback to Driver Hub
            robotTelemetry();

        }



    }



    public void robotDoStuff(){
        if(isStopRequested()) requestOpModeStop();
        follower.update();
        //ifs
        if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
            needsToExtraExtend = false;
            outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
        }
        if (follower.getVelocity().getMagnitude() == 0 && follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
            follower.breakFollowing();
        }
        if(shouldIntakeGoDown && intakeGoDownTimer.getElapsedTime() > 400){
            shouldIntakeGoDown = false;
            intakeCabinDownCollecting();
        }
        if(shouldIntakeExtendForLateColect && intakeShouldExtendTimer.getElapsedTime() > 700){
            shouldIntakeExtendForLateColect = false;
            intakeExtended3out4();
        }


        //color stuff
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);

        //PID Stuff
        intakeMotorPower = NewPidsController.pidControllerIntake(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
        outakeMotorPower = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos, outakeLeftMotor.getCurrentPosition());

        if(intakeMotorPower < 0) intakeMotorPower *= 1.3;
        if(currentStateOfSampleInIntake == colorSensorOutty.correctSample) intakeMotorPower *= 1.2;

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
        tel.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        tel.addData("intakePivotServoPos",intakePivotServoPos);
        tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
        tel.addData("outtakeDirection",outakeMotorPower);
        tel.addData("outtakeCurrenPos",outakeLeftMotor.getCurrentPosition());
        tel.addData("sensor color",currentStateOfSampleInIntake);
        tel.addData("follower is busy",follower.isBusy());
        Drawing.drawDebug(follower);
        tel.update();
    }



}