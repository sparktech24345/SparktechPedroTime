package pedroPathing.tests;

import static pedroPathing.PositionStorage.PIDincrement;
import static pedroPathing.PositionStorage.autoTimer;
import static pedroPathing.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.PositionStorage.intakeTargetPos;
import static pedroPathing.PositionStorage.intakeTargetPosAdder;
import static pedroPathing.PositionStorage.outakeArmServoPosition;
import static pedroPathing.PositionStorage.outakeSampleRetracted;
import static pedroPathing.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.PositionStorage.resetStuff;
import static pedroPathing.PositionStorage.servoextended;
import static pedroPathing.PositionStorage.stopMulthiread;

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

import pedroPathing.ControlMotor;
import pedroPathing.States.IntakeFSM;
import pedroPathing.States.IntakeStateExtendedHM;
import pedroPathing.States.IntakeStateExtendedRo2v2;
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


@Autonomous(name = "AutonomiePedroFOURSpecIntakeT2", group = "Examples")
@Disabled
public class AutonomiePedro4SpecIntakeT2 extends OpMode {
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
    IntakeStateExtendedRo2v2 intakeExtendedRo2v2Auto = new IntakeStateExtendedRo2v2();
    IntakeStateExtendedHM intakeExtendedRo2v2HM = new IntakeStateExtendedHM();
    IntakeStateWallPURetraction intakeStateWallPURetraction = new IntakeStateWallPURetraction();

    // Create the Outtake FSM with the initial state
    OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeStateStandbyWithSampleUp);

    // Create the Intake FSM with the initial state
    IntakeFSM intakeFSM = new IntakeFSM(intakeStateWallPURetraction);




    /**                         Our Paths!                          */
    private int pathState;
    private final int subtractor = 0;
    private final Pose startPose = new Pose(0, 58, Math.toRadians(0)); //start
    private final Pose startSpecimenPose = new Pose(24-subtractor, 68, Math.toRadians(0)); //line 1
    private final Pose intermediaryPos1=new Pose(27-subtractor,32,Math.toRadians(124)); //line 2 //old 124
    private final Pose intermediaryPos11=new Pose(27.2-subtractor,32.2,Math.toRadians(124)); //line 2
    private final Pose sample1LeftPose =new Pose(27-subtractor,28,Math.toRadians(40)); //line 3
    private final Pose sample1MovePose=new Pose(27-subtractor,22,Math.toRadians(124)); //line 4
    private final Pose sample11MovePose=new Pose(27.3-subtractor,22.3,Math.toRadians(124)); //line 4
    private final Pose sample1ObservationZonePose =new Pose(26-subtractor,18,Math.toRadians(30)); //line 5
    private final Pose sample2ObservationZonePose =new Pose(18-subtractor,20,Math.toRadians(0)); //line 8
    // private final Pose sample3LeftPose =new Pose(62,12,Math.toRadians(0)); //line 9
    // private final Pose sample3MovePose=new Pose(62,6,Math.toRadians(0)); //line 10
    // private final Pose sample3ObservationZonePose =new Pose(15,6,Math.toRadians(0)); //line 11//
    private final Pose getSpecimenPose=new Pose(0.1-subtractor,20,Math.toRadians(0));// line 12
    private final Pose beforeGetSpecimenPose=new Pose(5-subtractor,20,Math.toRadians(0));// line 12
    private final Pose ScoreSpecimenPose=new Pose(25-subtractor,60,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen1Score = new Pose(25-subtractor,56,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen2Score = new Pose(25-subtractor,58,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen3Score = new Pose(25-subtractor,62,Math.toRadians(0)); //all the same, line 13
    private final Pose pocketSpecimenPose = ScoreSpecimenPose;
    private final Pose parkingPose=new Pose(5-subtractor,5,Math.toRadians(0)); //parking

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;

    ExecutorService executorService = Executors.newFixedThreadPool(2);



    private Path forward,parking;
    private PathChain temp1,temp2,moveSample1a,moveSample1b,moveSample1c,moveSample1d,moveSample2c,moveSample3a,pickUpSpecimen1,pickUpSpecimen2,pickUpSpecimen3, scoreSpecimen1, scoreSpecimen2, scoreSpecimen3,pocketSpecimen,stand1,stand11,stand2,stand22,stand3,stand33;


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
        forward = new Path(new BezierLine(new Point(startPose), new Point(startSpecimenPose)));
        forward.setLinearHeadingInterpolation(startPose.getHeading(), startSpecimenPose.getHeading());

        moveSample1a=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(startSpecimenPose), new Point(intermediaryPos11)))
                .setLinearHeadingInterpolation(startSpecimenPose.getHeading(), intermediaryPos11.getHeading())

                .build();

        stand1=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(intermediaryPos11), new Point(intermediaryPos1)))
                .setLinearHeadingInterpolation(intermediaryPos11.getHeading(), intermediaryPos1.getHeading())
                .build();
        /*stand11=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(intermediaryPos11), new Point(intermediaryPos1)))
                .setLinearHeadingInterpolation(intermediaryPos11.getHeading(), intermediaryPos1.getHeading())
                .build();//*/


        moveSample1b=follower.pathBuilder()
                //goes from the intermediete pose (submersible corner) to sample1pose wich is to the top left of the actual sample
                .addPath(new BezierLine(new Point(intermediaryPos1),new Point(sample1LeftPose)))
                .setLinearHeadingInterpolation(intermediaryPos1.getHeading(), sample1LeftPose.getHeading())
                .build();

        moveSample1c=follower.pathBuilder()
                //goes from corner of sample to in front of it
                .addPath(new BezierLine(new Point(sample1LeftPose),new Point(sample11MovePose)))
                .setLinearHeadingInterpolation(sample1LeftPose.getHeading(),sample11MovePose.getHeading())
                .build();

        stand2=follower.pathBuilder()
                //goes from corner of sample to in front of it
                .addPath(new BezierLine(new Point(sample11MovePose),new Point(sample1MovePose)))
                .setLinearHeadingInterpolation(sample11MovePose.getHeading(),sample1MovePose.getHeading())
                .build();
        /*stand22=follower.pathBuilder()
                //goes from corner of sample to in front of it
                .addPath(new BezierLine(new Point(sample11MovePose),new Point(sample1MovePose)))
                .setLinearHeadingInterpolation(sample11MovePose.getHeading(),sample1MovePose.getHeading())
                .build();//*/




        moveSample1d=follower.pathBuilder()
                //pushes the sample in the oservation zone
                .addPath(new BezierLine(new Point(sample1MovePose),new Point(sample1ObservationZonePose)))
                .setLinearHeadingInterpolation(sample1MovePose.getHeading(), sample1ObservationZonePose.getHeading())
                .build();

        moveSample2c=follower.pathBuilder()
                //push sample 2 in the observation zone
                .addPath(new BezierLine(new Point(sample1ObservationZonePose),new Point(sample2ObservationZonePose)))
                .setLinearHeadingInterpolation(sample1ObservationZonePose.getHeading(), sample2ObservationZonePose.getHeading())
                .build();

        moveSample3a=follower.pathBuilder()
                //push sample 2 in the observation zone
                .addPath(new BezierLine(new Point(sample2ObservationZonePose),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(sample2ObservationZonePose.getHeading(), getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen1=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen1Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen1Score.getHeading())
                .build();

        scoreSpecimen1=follower.pathBuilder()
                //Go from bar after puttin Specimen 1 to the specimen colect pose
                .addPath(new BezierLine(new Point(specimen1Score),new Point(beforeGetSpecimenPose)))
                .setLinearHeadingInterpolation(specimen1Score.getHeading(),beforeGetSpecimenPose.getHeading())
                .build();

        temp1 =follower.pathBuilder()
                //Go from bar after puttin Specimen 1 to the specimen colect pose
                .addPath(new BezierLine(new Point(beforeGetSpecimenPose),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(beforeGetSpecimenPose.getHeading(),getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen2=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen2Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen2Score.getHeading())
                .build();

        scoreSpecimen2=follower.pathBuilder()
                //Go from bar after puttin Specimen 2 to the specimen colect pose
                .addPath(new BezierLine(new Point(specimen2Score),new Point(beforeGetSpecimenPose)))
                .setLinearHeadingInterpolation(specimen2Score.getHeading(),beforeGetSpecimenPose.getHeading())
                .build();

        temp2 =follower.pathBuilder()
                //Go from bar after puttin Specimen 2 to the specimen colect pose
                .addPath(new BezierLine(new Point(beforeGetSpecimenPose),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(beforeGetSpecimenPose.getHeading(),getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen3=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen3Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen3Score.getHeading())
                .build();

        //Go from bar after puttin Specimen 4 to parking and end path
        parking = new Path(new BezierLine(new Point(specimen3Score), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(specimen3Score.getHeading(), parkingPose.getHeading());

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        switch (pathState) {
            case 68:
                if(!follower.isBusy()) {
                    follower.followPath(forward,true);
                    setPathState(1);
                    outtakeFSM.setState(outtakeSpecimen);
                    outtakeFSM.executeCurrentState();
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    outtakeFSM.setState(outtakeSpecimenHang);
                    outtakeFSM.executeCurrentState();
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 350 > System.currentTimeMillis()){}
                    outakeSampleServoPosition = servoextended;

                follower.followPath(moveSample1a, true);
                setPathState(50);
                }
                break;

            case 50:
                if (!follower.isBusy()) {
                    follower.followPath(stand1,true);
                    setPathState(2);
                }
                break;

            /*case 51:
                if (!follower.isBusy()) {
                    follower.followPath(stand11,true);
                    setPathState(2);
                }
                break;//*/



            case 2:
                if (!follower.isBusy()) {

                    autoTimer = System.currentTimeMillis();
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                        if((colors.red >= 0.0012 || colors.blue >= 0.0012)) {
                            intakeMotorPickUpPower = 0;
                        }
                        }
                    follower.followPath(moveSample1b, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeMotorPickUpPower =-0.8;
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    intakeMotorPickUpPower =0;
                    autoTimer = System.currentTimeMillis();
                    //while(autoTimer + 200 > System.currentTimeMillis()){}
                    intakeTargetPos = 150;
                    follower.followPath(moveSample1c, true);
                    setPathState(52);
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    follower.followPath(stand2);
                    setPathState(4);
                }
                break;

            /*case 53:
                if (!follower.isBusy()) {
                    follower.followPath(stand22);
                    setPathState(4);
                }
                break;//*/


            case 4:
                if (!follower.isBusy()) {

                    autoTimer = System.currentTimeMillis();
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                        if((colors.red >= 0.0012 || colors.blue >= 0.0012)) {
                            intakeMotorPickUpPower = 0;
                        }
                    }
                    follower.followPath(moveSample1d, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeMotorPickUpPower =-0.8;
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    intakeMotorPickUpPower =0;
                    autoTimer = System.currentTimeMillis();
                    //while(autoTimer + 200 > System.currentTimeMillis()){}
                    intakeTargetPos = 0;
                    intakeFSM.setState(intakeStateWallPURetraction);
                    intakeFSM.executeCurrentState();
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();


                    follower.followPath(moveSample2c, true);
                    setPathState(8);
                }
                break;


            /*case 54:
                if (!follower.isBusy()) {
                    follower.followPath(moveSample2c);
                    setPathState(8);
                }
                break;

            /*case 55:
                if (!follower.isBusy()) {
                    follower.followPath(stand33);
                    setPathState(6);
                }
                break;//*/


            /*case 6:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    intakeFSM.setState(intakeExtendedRo2v2Auto);
                    intakeFSM.executeCurrentState();
                    outtakeFSM.setState(outtakeStateStandbyWithSampleUp);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 1500 > System.currentTimeMillis() && (colors.red <= 0.0012 && colors.blue <= 0.0012)){
                        colors = colorSensor.getNormalizedColors();
                        if(autoTimer + 200 < System.currentTimeMillis()) {
                            intakeTargetPos = 510;
                        }
                        if(autoTimer + 300 < System.currentTimeMillis()) {
                            intakeMotorPickUpPower = 1;
                        }
                        if((colors.red >= 0.0012 || colors.blue >= 0.0012)) {
                            intakeMotorPickUpPower = 0;
                        }
                    }
                    intakeTargetPos = 0;
                    follower.followPath(moveSample2b, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {

                    autoTimer = System.currentTimeMillis();
                    intakeMotorPickUpPower =-0.8;
                    while(autoTimer + 200 > System.currentTimeMillis()){}
                    intakeMotorPickUpPower =0;
                    autoTimer = System.currentTimeMillis();
                    intakeRotateServoPosition = intakeRotateForWallPickUp;
                    //while(autoTimer + 200 > System.currentTimeMillis()){}


                    intakeFSM.setState(intakeStateWallPURetraction);
                    intakeFSM.executeCurrentState();
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();


                    follower.followPath(moveSample2c, true);
                    setPathState(8);
                }
                break;//*/

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(moveSample3a, true);
                    setPathState(12);
                }
                break;

            /*case 9:
                if (!follower.isBusy()) {
                    follower.followPath(moveSample3b, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(moveSample3c, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(moveSample3d, true);
                    setPathState(12);
                }
                break;             //*/
            case 12:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outakeSampleServoPosition= outakeSampleRetracted;
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeSpecimenAuto);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(pickUpSpecimen1, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outtakeFSM.setState(outtakeSpecimenHangAuto);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 350 > System.currentTimeMillis()){}
                    outakeSampleServoPosition = servoextended;
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(scoreSpecimen1);
                    setPathState(61);
                }
                break;

            case 61:
                if (!follower.isBusy()) {
                    follower.followPath(temp1, true);
                    setPathState(14);
                }

            case 14:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outakeSampleServoPosition= outakeSampleRetracted;
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeSpecimenAuto);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(pickUpSpecimen2,true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outtakeFSM.setState(outtakeSpecimenHangAuto);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 350 > System.currentTimeMillis()){}
                    outakeSampleServoPosition = servoextended;
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(scoreSpecimen2);
                    setPathState(60);
                }
                break;

            case 60:
                if (!follower.isBusy()) {
                    follower.followPath(temp2, true);
                    setPathState(16);
                }

            case 16:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outakeSampleServoPosition= outakeSampleRetracted;
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outtakeSpecimenAuto);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(pickUpSpecimen3);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    autoTimer = System.currentTimeMillis();
                    outtakeFSM.setState(outtakeSpecimenHangAuto);
                    outtakeFSM.executeCurrentState();
                    while(autoTimer + 350 > System.currentTimeMillis()){}
                    outakeSampleServoPosition = servoextended;
                    autoTimer = System.currentTimeMillis();
                    while(autoTimer + 50 > System.currentTimeMillis()){}
                    outtakeFSM.setState(outakeHMandWallPU);
                    outtakeFSM.executeCurrentState();
                    follower.followPath(parking);
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
        resetStuff();

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
        resetStuff();

        FollowerConstants.pathEndTimeoutConstraint = 500;
        Constants.setConstants(FConstants.class, LConstants.class);
        FollowerConstants.pathEndTimeoutConstraint = 500;
        follower = new Follower(hardwareMap);
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

                stopMulthiread = false;
                while(!stopMulthiread){
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

                    /*telemetry.addData("frontLeftPowerCat",frontLeftPowerCat);
                    telemetry.addData("backLeftPowerCat",backLeftPowerCat);
                    telemetry.addData("frontRightPowerCat",frontRightPowerCat);
                    telemetry.addData("backRightPowerCat",backRightPowerCat);

                    telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    updateTelemetry(telemetry);
                       //*/
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
        executorService.shutdownNow();
    }
}