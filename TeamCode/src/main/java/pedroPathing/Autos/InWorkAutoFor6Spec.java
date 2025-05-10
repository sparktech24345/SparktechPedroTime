package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "6SpecHopeyHopey", group = "Examples")
public class InWorkAutoFor6Spec extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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

    private final Pose startPose = new Pose(-10, 70, Math.toRadians(90)); //start
    private final Pose scoringBarPose = new Pose(-5.4, 41, Math.toRadians(90)); //start
    private final Pose specimenPickUpPose = new Pose(-43, 69.6, Math.toRadians(90)); //start
    private final Pose firstSamplePickUpPos = new Pose(-50, 49, Math.toRadians(90)); //start
    private final Pose secondSamplePickUpPos = new Pose(-61, 49, Math.toRadians(90)); //start
    private final Pose thirdSamplePickUpPos = new Pose(-60.5, 49, Math.toRadians(125)); //start
    private final Pose parkingPose=new Pose(27,82,Math.toRadians(180)); //parking

    double intakeMotorPower=0;
    double outakeMotorPower=0;


    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;


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
        startPath = new Path(new BezierLine(new Point(startPose), new Point(scoringBarPose)));
        startPath.setLinearHeadingInterpolation(startPose.getHeading(), scoringBarPose.getHeading());

        goToPickUpFirstSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringBarPose), new Point(firstSamplePickUpPos)))
                .setLinearHeadingInterpolation(scoringBarPose.getHeading(), firstSamplePickUpPos.getHeading())
                .build();

        goToPickUpSecondSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSamplePickUpPos), new Point(secondSamplePickUpPos)))
                .setLinearHeadingInterpolation(firstSamplePickUpPos.getHeading(), secondSamplePickUpPos.getHeading())
                .build();

        goToPickUpThirdSample=follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePickUpPos), new Point(thirdSamplePickUpPos)))
                .setLinearHeadingInterpolation(secondSamplePickUpPos.getHeading(), thirdSamplePickUpPos.getHeading())
                .build();


        //first spec
        pickUpFirst = new Path(new BezierLine(new Point(thirdSamplePickUpPos), new Point(specimenPickUpPose)));
        pickUpFirst.setLinearHeadingInterpolation(thirdSamplePickUpPos.getHeading(), specimenPickUpPose.getHeading());

        scoreFirst = new Path(new BezierLine(new Point(specimenPickUpPose), new Point(scoringBarPose)));
        scoreFirst.setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), scoringBarPose.getHeading());


        //second spec

        pickUpSecond = new Path(new BezierLine(new Point(scoringBarPose), new Point(specimenPickUpPose)));
        pickUpSecond.setLinearHeadingInterpolation(scoringBarPose.getHeading(), specimenPickUpPose.getHeading());

        scoreSecond = new Path(new BezierLine(new Point(specimenPickUpPose), new Point(scoringBarPose)));
        scoreSecond.setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), scoringBarPose.getHeading());


        //third spec

        pickUpThird = new Path(new BezierLine(new Point(scoringBarPose), new Point(specimenPickUpPose)));
        pickUpThird.setLinearHeadingInterpolation(scoringBarPose.getHeading(), specimenPickUpPose.getHeading());

        scoreThird = new Path(new BezierLine(new Point(specimenPickUpPose), new Point(scoringBarPose)));
        scoreThird.setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), scoringBarPose.getHeading());


        //fourth spec

        pickUpFourth = new Path(new BezierLine(new Point(scoringBarPose), new Point(specimenPickUpPose)));
        pickUpFourth.setLinearHeadingInterpolation(scoringBarPose.getHeading(), specimenPickUpPose.getHeading());

        scoreFourth = new Path(new BezierLine(new Point(specimenPickUpPose), new Point(scoringBarPose)));
        scoreFourth.setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), scoringBarPose.getHeading());


        //fifth spec

        pickUpFifth = new Path(new BezierLine(new Point(scoringBarPose), new Point(specimenPickUpPose)));
        pickUpFifth.setLinearHeadingInterpolation(scoringBarPose.getHeading(), specimenPickUpPose.getHeading());

        scoreFifth = new Path(new BezierLine(new Point(specimenPickUpPose), new Point(scoringBarPose)));
        scoreFifth.setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), scoringBarPose.getHeading());




        //parking
        parking = new Path(new BezierLine(new Point(scoringBarPose), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(scoringBarPose.getHeading(), parkingPose.getHeading());

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
                    autoOuttakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(startPath,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(goToPickUpFirstSample,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    intakeExtended2out4();
                    //if()
                    follower.followPath(goToPickUpSecondSample,true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(goToPickUpThirdSample,true);
                    setPathState(105);
                }
                break;

            //first spec
            case 105:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    waitWhile(150);
                    outtakeWallPickUpNew();
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
                    outtakeWallPickUpNew();
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
                    outtakeWallPickUpNew();
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
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
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

            //fifth spec
            case 113:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(pickUpFifth,true);
                    setPathState(114);
                }
                break;
            case 114:
                if(!follower.isBusy()) {
                    outtakeClawServoPos = outtakeClawServoRetractedPos;
                    waitWhile(150);
                    outtakeSpecimenHang();
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(scoreFifth,true);
                    setPathState(115);
                }
                break;

            case 115:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    follower.followPath(parking,true);
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


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

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        // Set init position
        initStates();
        intakeRotateServo.setPosition(intakePivotServoPos / 228);
        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);
        //end of our stuff
    }


    public void robotDoStuff(){

        //PID Stuff
        intakeMotorPower = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
        outakeMotorPower = outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos, outakeLeftMotor.getCurrentPosition());

        //set motor positions
        intakeMotor.setPower(intakeMotorPower);
        outakeRightMotor.setPower(outakeMotorPower);
        outakeLeftMotor.setPower(outakeMotorPower);
        intakeSpinMotor.setPower(intakeSpinMotorPow);

        //Set servo Positions
        intakeRotateServo.setPosition((intakePivotServoPos) / 360);
        outakeArmServo.setPosition(outtakePivotServoPos / 360);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        robotDoStuff();


        // Feedback to Driver Hub
        tel.addData("path state", pathState);
        tel.addData("x", follower.getPose().getX());
        tel.addData("y", follower.getPose().getY());
        tel.addData("heading", follower.getPose().getHeading());
        tel.addData("intakePivotServoPos",intakePivotServoPos);
        tel.addData("outtakeTargetPos",outtakeExtendMotorTargetPos);
        tel.addData("outtakeDirection",outakeMotorPower);
        tel.addData("outtakeCurrenPos",outakeLeftMotor.getCurrentPosition());
        Drawing.drawDebug(follower);
        tel.update();
    }



    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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