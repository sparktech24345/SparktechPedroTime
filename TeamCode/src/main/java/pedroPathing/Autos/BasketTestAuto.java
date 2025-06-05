package pedroPathing.Autos;

import static pedroPathing.ClassWithStates.*;
import static pedroPathing.OrganizedPositionStorage.*;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "BasketTestAuto", group = "Examples")
public class BasketTestAuto extends OpMode {
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
    private final Pose behindBasketPreload = new Pose(-41.2, 80.2, Math.toRadians(225));
    private final Pose behindBasketFirstSample = new Pose(-48.5 , 88, Math.toRadians(225));
    private final Pose firstSampleCollect = new Pose(-47, 92, 5.3);
    private final Pose secondSampleCollect = new Pose(-46.6, 91, 5.3);
    private final Pose thirdSampleCollect = new Pose(-46.6, 91, 5.3);



    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    //private Path startPath,pickUpFirst,pickUpSecond,pickUpThird,pickUpFourth,pickUpFifth,scoreFirst,scoreSecond,scoreThird,scoreFourth,scoreFifth,parking;
    //private PathChain goToPickUpFirstSample,goToPickUpSecondSample,goToPickUpThirdSample;

    private PathChain firstSampleCollectPath, preloadScorePath, firstSampleScorePath, secondSampleCollectPath, secondSampleScorePath, thirdSampleCollectPath, thirdSampleScorePath;

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

        /*
        /// SECOND SAMPLE

        secondSampleCollectPath = new Path(new BezierLine(new Point(behindBasketPreload), new Point(secondSampleCollect)));
        secondSampleCollectPath.setLinearHeadingInterpolation(behindBasketPreload.getHeading(), secondSampleCollect.getHeading());

        secondSampleScorePath = new Path(new BezierLine(new Point(secondSampleCollect), new Point(behindBasketPreload)));
        secondSampleScorePath.setLinearHeadingInterpolation(secondSampleCollect.getHeading(), behindBasketPreload.getHeading());

        /// THIRD SAMPLE

        thirdSampleCollectPath = new Path(new BezierLine(new Point(behindBasketPreload), new Point(thirdSampleCollect)));
        thirdSampleCollectPath.setLinearHeadingInterpolation(behindBasketPreload.getHeading(), thirdSampleCollect.getHeading());

        thirdSampleScorePath = new Path(new BezierLine(new Point(thirdSampleCollect), new Point(behindBasketPreload)));
        thirdSampleScorePath.setLinearHeadingInterpolation(thirdSampleCollect.getHeading(), behindBasketPreload.getHeading());
        */
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    // from 1 - 100 is normal paths
    // from 100+ is scoring paths
    private final int slideExtensionTimer = 300;
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
                    autoTimer = System.currentTimeMillis();
                    intakeCabinDownCollecting();
                    waitWhile(500);
                    autoOuttakeTransfer();
                    waitWhile(300);
                    intakeExtended4out4();
                    while(!(currentStateOfSampleInIntake == colorSensorOutty.wrongSample || currentStateOfSampleInIntake == colorSensorOutty.correctSample)) robotDoStuff();
                    intakeRetracted();
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    executeAutoTransfer();
                    waitWhile(500);
                    follower.followPath(firstSampleScorePath);
                    //outtakeClawServoPos = outtakeClawServoExtendedPos;
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    waitWhile(300);
                    outtakeClawServoPos = outtakeClawServoExtendedPos;
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
    }

    public void executeAutoTransfer() {
        waitWhile(500);
        intakeSpinMotorPow = 0;
        intakeCabinTransferPosition();
        waitWhile(500);
        outtakeClawServoPos = outtakeClawServoRetractedPos;
        waitWhile(200);
        outtakeBasket();
    }

    public void robotDoStuff(){
        //risky
        follower.update();




        //ifs
        if(needsToExtraExtend && outtakeIsInNeedToExtraExtendClawTimer + 400 < System.currentTimeMillis()){
            needsToExtraExtend = false;
            outtakeClawServoPos = outtakeClawServoExtraExtendedPos;
        }

        

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
        if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
            currentTeam = colorList.blue;
        if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
            currentTeam = colorList.red;
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
