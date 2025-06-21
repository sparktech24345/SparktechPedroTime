package pedroPathing.tests;

import static pedroPathing.newOld.PositionStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.AutoPIDS.ControlMotor;
import pedroPathing.States.IntakeFSM;
import pedroPathing.States.IntakeStateExtendedHM;
import pedroPathing.States.IntakeStateExtendedRo2v2Auto;
import pedroPathing.States.IntakeStateRetractedRo2;
import pedroPathing.States.IntakeStateWallPURetraction;
import pedroPathing.States.OutakeHMandWallPU;
import pedroPathing.States.OuttakeFSM;
import pedroPathing.States.OuttakeSpecimenHang;
import pedroPathing.States.OuttakeSpecimenHangAuto;
import pedroPathing.States.OuttakeStateBasket;
import pedroPathing.States.OuttakeStateSamplePickUp;
import pedroPathing.States.OuttakeStateSpecimen;
import pedroPathing.States.OuttakeStateSpecimenAuto;
import pedroPathing.States.OuttakeStateStandbyDownWithSample;
import pedroPathing.States.OuttakeStateStandbyWithSampleUp;
import pedroPathing.States.OuttakeStateTranfer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "AutonomiePedroBasketasdasdasdasdasda", group = "Examples")
@Disabled
public class AutonomiePedroBasketOldConcent extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    // Initialize Outtake states
    OuttakeStateSpecimenAuto outtakeSpecimenAuto = new OuttakeStateSpecimenAuto();
    OuttakeStateSpecimen outtakeSpecimen = new OuttakeStateSpecimen();
    OuttakeSpecimenHang outtakeSpecimenHang = new OuttakeSpecimenHang();
    OuttakeSpecimenHangAuto outtakeSpecimenHangAuto = new OuttakeSpecimenHangAuto();
    OuttakeStateBasket outtakeBasket = new OuttakeStateBasket();
    OuttakeStateSamplePickUp outtakeSamplePickUp = new OuttakeStateSamplePickUp();
    OuttakeStateStandbyDownWithSample outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
    OuttakeStateTranfer outtakeStateTranfer = new OuttakeStateTranfer();
    OuttakeStateStandbyWithSampleUp outtakeStateStandbyWithSampleUp = new OuttakeStateStandbyWithSampleUp();
    OutakeHMandWallPU outakeHMandWallPU = new OutakeHMandWallPU();

    // Initialize Intake states
    IntakeStateRetractedRo2 intakeRetractedRo2 = new IntakeStateRetractedRo2();
    IntakeStateExtendedRo2v2Auto intakeExtendedRo2v2Auto = new IntakeStateExtendedRo2v2Auto();
    IntakeStateExtendedHM intakeExtendedRo2v2HM = new IntakeStateExtendedHM();
    IntakeStateWallPURetraction intakeStateWallPURetraction = new IntakeStateWallPURetraction();

    // Create the Outtake FSM with the initial state
    OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStateStandbyWithSampleUp);

    // Create the Intake FSM with the initial state
    IntakeFSM intakeFSM = new IntakeFSM(intakeStateWallPURetraction);














    /**                         Our Paths!                          */
    private int pathState;

    private final Pose startPose = new Pose(1, 70, Math.toRadians(90)); //start
    private final Pose FirstPose2Sample = new Pose(16, 94, Math.toRadians(150)); //line 1
    private final Pose ThirdSamplePose =new Pose(17.2,102,Math.toRadians(169)); //line 2 //old 124
    private final Pose FourthSamplePickUp=new Pose(21,100,Math.toRadians(201)); //line 3
    private final Pose FourthSampleScore =new Pose(21.2,102,Math.toRadians(169)); //line 4
    private final Pose intermediaryPoseBeforePark=new Pose(52.5,71,Math.toRadians(270)); //line 5
    private final Pose parkingPose=new Pose(52.5,82,Math.toRadians(270)); //parking

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;

    ExecutorService executorService = Executors.newFixedThreadPool(2);



    private Path goToFirstPoint,parking;
    private PathChain goToThirdSample,goToSecondPoint, goToFourthSamplePickUp,goToFourthSampleScore,goToPointBeforePark;

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
        goToFirstPoint = new Path(new BezierLine(new Point(startPose), new Point(FirstPose2Sample)));
        goToFirstPoint.setLinearHeadingInterpolation(startPose.getHeading(), FirstPose2Sample.getHeading());

        goToThirdSample =follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(FirstPose2Sample), new Point(ThirdSamplePose)))
                .setLinearHeadingInterpolation(FirstPose2Sample.getHeading(), ThirdSamplePose.getHeading())
                .build();
        goToFourthSamplePickUp =follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(ThirdSamplePose), new Point(FourthSamplePickUp)))
                .setLinearHeadingInterpolation(ThirdSamplePose.getHeading(), FourthSamplePickUp.getHeading())
                .build();
        goToFourthSampleScore =follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(FourthSamplePickUp), new Point(FourthSampleScore)))
                .setLinearHeadingInterpolation(FourthSamplePickUp.getHeading(), FourthSampleScore.getHeading())
                .build();

        goToPointBeforePark =follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(FourthSampleScore), new Point(intermediaryPoseBeforePark)))
                .setLinearHeadingInterpolation(FourthSampleScore.getHeading(), intermediaryPoseBeforePark.getHeading())
                .build();

        //Go from bar after puttin Specimen 4 to parking and end path
        parking = new Path(new BezierLine(new Point(intermediaryPoseBeforePark), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(intermediaryPoseBeforePark.getHeading(), parkingPose.getHeading());

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                    //while(autoTimer + 500 > System.currentTimeMillis()){}
                    follower.followPath(goToFirstPoint,true);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outakeSampleServoPosition = servoextended;
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    autoTimer = System.currentTimeMillis();
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                    }
                    outtakeFSM.setState(outtakeStateTranfer);
                    outtakeFSM.executeCurrentState();
                    autoTimer = System.currentTimeMillis();
                    while (autoTimer + 3000 > System.currentTimeMillis() && !didTransfer) {
                    }
                    if(didTransfer) {
                        //presumably after it has picked up and its transfering

                        //evade situatin where the rising sliders will hit basket with the servo alr there
                        outakeTargetPos = -2800;

                        //actual basket pose
                        autoTimer = System.currentTimeMillis();
                        while (autoTimer + 600 > System.currentTimeMillis()) {
                        }
                        autoTimer = System.currentTimeMillis();
                        outtakeFSM.setState(outtakeBasket);

                        //wait for stuff to happen stuff
                        autoTimer = System.currentTimeMillis();
                        while (autoTimer + 200 > System.currentTimeMillis()) {
                        }
                        outakeSampleServoPosition = servoextended;

                        //wait after release
                        autoTimer = System.currentTimeMillis();
                        while (autoTimer + 200 > System.currentTimeMillis()) {
                        }
                        if(!didTransfer) intakeTargetPos = intakeSlidersRo2Transfer;
                        didTransfer = false;
                    }
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();

                    follower.followPath(goToThirdSample,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();

                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                    }
                    outtakeFSM.setState(outtakeStateTranfer);
                    outtakeFSM.executeCurrentState();

                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 3000 > System.currentTimeMillis() && !didTransfer){}

                    if(didTransfer) {
                        outtakeFSM.setState(outtakeBasket);
                        outtakeFSM.executeCurrentState();

                        autoTimer = System.currentTimeMillis();
                        while (autoTimer + 1200 > System.currentTimeMillis()) {
                        }
                        outakeSampleServoPosition = servoextended;

                        //wait after release
                        autoTimer = System.currentTimeMillis();
                        while (autoTimer + 200 > System.currentTimeMillis()) {
                        }

                        outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                        outtakeFSM.executeCurrentState();
                        if(!didTransfer) intakeTargetPos = intakeSlidersRo2Transfer;
                        didTransfer = false;
                    }

                    follower.followPath(goToFourthSamplePickUp, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 1000 > System.currentTimeMillis()){}
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();

                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                    }
                    outtakeFSM.setState(outtakeStateTranfer);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(goToFourthSampleScore,true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    outtakeFSM.setState(outtakeBasket);
                    outtakeFSM.executeCurrentState();
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 2000 > System.currentTimeMillis()){}
                    outakeSampleServoPosition = servoextended;

                    //wait after release
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    outakeArmServoPosition = 250;
                    follower.followPath(goToPointBeforePark,false);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                follower.followPath(parking,false);
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

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("intakerotate",intakeRotateServoPosition);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        FollowerConstants.pathEndTimeoutConstraint = 2000;
        Constants.setConstants(FConstants.class, LConstants.class);
        FollowerConstants.pathEndTimeoutConstraint = 2000;
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        intakeFSM.setState(outakeHMandWallPU);
        intakeFSM.executeCurrentState();
        outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
        outtakeFSM.executeCurrentState();
        intakeRotateServoPosition = 128;


        // our stuff
        executorService.execute(new Runnable() {
            @Override
            public void run() {

                stopMulthiread = false;

                double intakeMotorPower = 0;
                double outakeMotorPower = 0;
                DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
                DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
                DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
                DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                intakeControlMotor = new ControlMotor();
                outakeControlMotor = new ControlMotor();
                //servos
                outakeSampleServoPosition = outakeSampleRetracted;
                Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
                Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
                Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");

                NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

                NormalizedRGBA colors = colorSensor.getNormalizedColors();


                while(!stopMulthiread){
                    colors = colorSensor.getNormalizedColors();
                    intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
                    outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
                    outakeMotorPower *= PIDincrement;

                    intakeMotor.setPower(intakeMotorPower);
                    outakeRightMotor.setPower(outakeMotorPower);
                    outakeLeftMotor.setPower(outakeMotorPower);
                    intakeSpinMotor.setPower(intakeMotorPickUpPower);


                    //Set servo Positions
                    intakeRotateServo.setPosition((intakeRotateServoPosition) / 360);
                    outakeArmServo.setPosition(outakeArmServoPosition / 360);
                    outakeSampleServo.setPosition(outakeSampleServoPosition / 360);

                    //telemetry.addData("frontLeftPowerCat",frontLeftPowerCat);
                    // telemetry.addData("backLeftPowerCat",backLeftPowerCat);
                    // telemetry.addData("frontRightPowerCat",frontRightPowerCat);
                    // telemetry.addData("backRightPowerCat",backRightPowerCat);
                    // telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    // updateTelemetry(telemetry);//


                    //transfer
                    if (((outakeArmServo.getPosition() * 360 <= outtakeArmServoPosAtRo2v2TransferPickUp + 5))
                            && outtakeFSM.currentStateOutake == outtakeStateTranfer
                            && (colors.red >= 0.0015 || colors.blue >= 0.0015)
                            && !transferDisabled
                            || gamepad1.right_trigger >= 0.4
                    ) {

                        //start timer
                        if (wasBambuExtended) {
                            bambuTransferTimer = System.currentTimeMillis();
                            wasBambuExtended = false;
                            doOnceyTransfer = true;
                            someExtraThingDoOnce = true;
                        }

                        //ACTUAL TRANSFER
                        //claw down
                        if (bambuTransferTimer + 200 < System.currentTimeMillis()) {
                            if (doOnceyTransfer) intakeRotateServoPosition = intakeRo2SmashPos;
                            if (doOnceyTransfer && bambuTransferTimer + 600 < System.currentTimeMillis()) {
                                intakeFSM.setState(intakeRetractedRo2);
                                intakeFSM.executeCurrentState();
                                doOnceyTransfer = false;
                                intakeExtraSpinOUTPUTTimer = System.currentTimeMillis();
                                intakeExtraSpinOUTPUTDoOnce = true;
                                //intakeMotorPickUpPower = -0.5;
                                DontDoTransferBeforeTransfer = true;
                            }
                            if ((intakeMotor.getCurrentPosition() <= intakeTargetPosAdder + intakeTargetPos + 8) && someExtraThingDoOnce && bambuTransferTimer + 600 < System.currentTimeMillis()) {
                                noWiglyTransferTimer = System.currentTimeMillis();
                                noWiglyPls = true;
                                someExtraThingDoOnce = false;
                            }
                        }
                        //close thingyy
                        if (noWiglyPls && noWiglyTransferTimer + 200 < System.currentTimeMillis()) {
                            outakeSampleServoPosition = outakeSampleRetracted;
                        }
                        //transfer done
                        if (noWiglyPls && noWiglyTransferTimer + 500 < System.currentTimeMillis() && DontDoTransferBeforeTransfer) {
                            outtakeFSM.setState(outtakeStandbyDown);
                            outtakeFSM.executeCurrentState();
                            outakeSampleServoPosition = outakeSampleRetracted;
                            //intakeRotateServoPosition = intakeRotateAfterRo2Trasfer;
                            extendABitAfterRo2Transfer = true;
                            intakeShouldRetractAfterTransfer = true;
                            noWiglyPls = false;
                            DontDoTransferBeforeTransfer = false;
                            didTransfer = true;
                        }
                    }
                    if (extendABitAfterRo2Transfer && !transferDisabled) {
                        intakeTargetPos = extendABitAfterRo2TransferPos;
                        extendABitAfterRo2Transfer = false;
                    }
                    if (intakeShouldRetractAfterTransfer && outakeArmServo.getPosition() * 360 >= outakeArmTransferPos - 10 && !transferDisabled) {
                        intakeShouldRetractAfterTransfer = false;
                        intakeShouldRetractAfterTransferTimerToggle = true;
                        intakeShouldRetractAfterTransferTimer = System.currentTimeMillis();
                    }
                    if (intakeShouldRetractAfterTransferTimerToggle && intakeShouldRetractAfterTransferTimer + 400 < System.currentTimeMillis() && !((colors.red >= 0.0015 || colors.blue >= 0.0015)) && !transferDisabled) {
                        intakeFSM.setState(intakeRetractedRo2);
                        intakeFSM.executeCurrentState();
                        intakeShouldRetractAfterTransferTimerToggle = false;
                    }
                    //*/

                    //telemetry.addData("path state", pathState);
                    //telemetry.addData("x", follower.getPose().getX());
                    //telemetry.addData("y", follower.getPose().getY());
                    //telemetry.addData("heading", follower.getPose().getHeading());
                    //telemetry.addData("intakerotate",intakeRotateServoPosition);
                    //telemetry.update();
                }
            }
        });
        //*/
        //end of our stuff
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
        stopMulthiread = true;
    }




}