package pedroPathing.tests;

import static pedroPathing.States.PositionStorage.PIDincrement;
import static pedroPathing.States.PositionStorage.backLeftPowerCat;
import static pedroPathing.States.PositionStorage.backRightPowerCat;
import static pedroPathing.States.PositionStorage.frontLeftPowerCat;
import static pedroPathing.States.PositionStorage.frontRightPowerCat;
import static pedroPathing.States.PositionStorage.gravityAdder;
import static pedroPathing.States.PositionStorage.intakeMotorPickUpPower;
import static pedroPathing.States.PositionStorage.intakeRotateServoPosition;
import static pedroPathing.States.PositionStorage.outakeArmServoPosition;
import static pedroPathing.States.PositionStorage.outakeSampleRetracted;
import static pedroPathing.States.PositionStorage.outakeSampleServoPosition;
import static pedroPathing.States.PositionStorage.outakeTargetPos;
import static pedroPathing.States.PositionStorage.outakeTargetPosAdder;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.ControlMotor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "AutonomiePedroTester", group = "Examples")
public class AutonomiePedroTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**                         Our Paths!                          */
    private int pathState;

    private final Pose startPose = new Pose(1, 68, Math.toRadians(0)); //start
    private final Pose startSpecimenPose = new Pose(35, 68, Math.toRadians(0)); //line 1
    private final Pose intermediaryPos1=new Pose(35,34,Math.toRadians(0)); //line 2
    private final Pose sample1LeftPose =new Pose(62,34,Math.toRadians(0)); //line 3
    private final Pose sample1MovePose=new Pose(62,24,Math.toRadians(0)); //line 4
    private final Pose sample1ObservationZonePose =new Pose(15,24,Math.toRadians(0)); //line 5
    private final Pose sample2LeftPose =new Pose(62,24,Math.toRadians(0)); //line 6
    private final Pose sample2MovePose=new Pose(62,12,Math.toRadians(0)); //line 7
    private final Pose sample2ObservationZonePose =new Pose(15,12,Math.toRadians(0)); //line 8
    private final Pose sample3LeftPose =new Pose(62,12,Math.toRadians(0)); //line 9
    private final Pose sample3MovePose=new Pose(62,4,Math.toRadians(0)); //line 10
    private final Pose sample3ObservationZonePose =new Pose(15,4,Math.toRadians(0)); //line 11
    private final Pose getSpecimenPose=new Pose(2,15,Math.toRadians(0));// line 12
    private final Pose ScoreSpecimenPose=new Pose(34,34,Math.toRadians(0)); //all the same, line 13
    private final Pose specimen1Score = ScoreSpecimenPose;
    private final Pose specimen2Score = ScoreSpecimenPose;
    private final Pose specimen3Score = ScoreSpecimenPose;
    private final Pose pocketSpecimenPose = ScoreSpecimenPose;
    private final Pose parkingPose=new Pose(5,5,Math.toRadians(0)); //parking

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;

    ExecutorService executorService = Executors.newFixedThreadPool(2);



    private Path forward,parking;
    private PathChain moveSample1, moveSample2, moveSample3,pickUpSpecimen1,pickUpSpecimen2,pickUpSpecimen3, scoreSpecimen1, scoreSpecimen2, scoreSpecimen3,pocketSpecimen;


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

        moveSample1=follower.pathBuilder()
                //goes from bar to an intermediete pose (submersible corner)
                .addPath(new BezierLine(new Point(startSpecimenPose), new Point(intermediaryPos1)))
                .setLinearHeadingInterpolation(startSpecimenPose.getHeading(), intermediaryPos1.getHeading())

                //goes from the intermediete pose (submersible corner) to sample1pose wich is to the top left of the actual sample
                .addPath(new BezierLine(new Point(intermediaryPos1),new Point(sample1LeftPose)))
                .setLinearHeadingInterpolation(intermediaryPos1.getHeading(), sample1LeftPose.getHeading())

                //goes from corner of sample to in front of it
                .addPath(new BezierLine(new Point(sample1LeftPose),new Point(sample1MovePose)))
                .setLinearHeadingInterpolation(sample1LeftPose.getHeading(),sample1MovePose.getHeading())

                //pushes the sample in the oservation zone
                .addPath(new BezierLine(new Point(sample1MovePose),new Point(sample1ObservationZonePose)))
                .setLinearHeadingInterpolation(sample1MovePose.getHeading(), sample1ObservationZonePose.getHeading())
                .build();

        moveSample2=follower.pathBuilder()
                //goes from the observation zone pose after pushing sample 1 to the top left cornener of the sample
                .addPath(new BezierLine(new Point(sample1ObservationZonePose), new Point(sample2LeftPose)))
                .setLinearHeadingInterpolation(sample1ObservationZonePose.getHeading(), sample2LeftPose.getHeading())

                //goes from the top left corner of the sample to in front of it
                .addPath(new BezierLine(new Point(sample2LeftPose),new Point(sample2MovePose)))
                .setLinearHeadingInterpolation(sample2LeftPose.getHeading(),sample2MovePose.getHeading())

                //push sample 2 in the observation zone
                .addPath(new BezierLine(new Point(sample2MovePose),new Point(sample2ObservationZonePose)))
                .setLinearHeadingInterpolation(sample2MovePose.getHeading(), sample2ObservationZonePose.getHeading())
                .build();


        moveSample3=follower.pathBuilder()
                //Go from OBS zone to the top left corner of the 3 sample
                .addPath(new BezierLine(new Point(sample2ObservationZonePose), new Point(sample3LeftPose)))
                .setLinearHeadingInterpolation(sample2ObservationZonePose.getHeading(), sample3LeftPose.getHeading())

                //go from the corner in front of the sample
                .addPath(new BezierLine(new Point(sample3LeftPose),new Point(sample3MovePose)))
                .setLinearHeadingInterpolation(sample3LeftPose.getHeading(),sample3MovePose.getHeading())

                //push the sample 3 in the OBS zone
                .addPath(new BezierLine(new Point(sample3MovePose),new Point(sample3ObservationZonePose)))
                .setLinearHeadingInterpolation(sample3MovePose.getHeading(), sample3ObservationZonePose.getHeading())

                //Go from where you left of with sample 3 to the specimen colection
                .addPath(new BezierLine(new Point(sample3ObservationZonePose),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(sample3ObservationZonePose.getHeading(), getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen1=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen1Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen1Score.getHeading())
                .build();

        scoreSpecimen1=follower.pathBuilder()
                //Go from bar after puttin Specimen 1 to the specimen colect pose
                .addPath(new BezierLine(new Point(specimen1Score),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(specimen1Score.getHeading(),getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen2=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen2Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen2Score.getHeading())
                .build();

        scoreSpecimen2=follower.pathBuilder()
                //Go from bar after puttin Specimen 2 to the specimen colect pose
                .addPath(new BezierLine(new Point(specimen2Score),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(specimen2Score.getHeading(),getSpecimenPose.getHeading())
                .build();

        pickUpSpecimen3=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose), new Point(specimen3Score)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(), specimen3Score.getHeading())
                .build();

        scoreSpecimen3=follower.pathBuilder()
                //Go from bar after puttin Specimen 3 to the specimen colect pose
                .addPath(new BezierLine(new Point(specimen3Score),new Point(getSpecimenPose)))
                .setLinearHeadingInterpolation(specimen3Score.getHeading(),getSpecimenPose.getHeading())
                .build();

        pocketSpecimen=follower.pathBuilder()
                //Go from the specimen colect pose to the specimen score pose
                .addPath(new BezierLine(new Point(getSpecimenPose),new Point(pocketSpecimenPose)))
                .setLinearHeadingInterpolation(getSpecimenPose.getHeading(),pocketSpecimenPose.getHeading())
                .build();

        //Go from bar after puttin Specimen 4 to parking and end path
        parking = new Path(new BezierLine(new Point(specimen3Score), new Point(parkingPose)));
        parking.setLinearHeadingInterpolation(specimen3Score.getHeading(), parkingPose.getHeading());

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(forward);
                setPathState(1);
                break;

            case 1:
                follower.followPath(moveSample1,true);
                setPathState(2);
                break;

            case 2:
                follower.followPath(moveSample2,true);
                setPathState(3);
                break;

            case 3:
                follower.followPath(moveSample3,true);
                setPathState(4);
                break;

            case 4:
                follower.followPath(pickUpSpecimen1,true);
                setPathState(5);


            case 5:
                follower.followPath(scoreSpecimen1);
                setPathState(6);
                break;
            case 6:
                follower.followPath(pickUpSpecimen2);
                setPathState(7);
                break;
            case 7:
                follower.followPath(scoreSpecimen2);
                setPathState(8);
                break;

            case 8:
                follower.followPath(pickUpSpecimen3);
                setPathState(9);
                break;

            case 9:
                follower.followPath(scoreSpecimen3);
                setPathState(10);
                break;

            case 10:
                follower.followPath(pocketSpecimen);
                setPathState(11);
                break;

            case 11:
                follower.followPath(parking);
                setPathState(-1);
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
        telemetry.update();
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
                //Servo tester = hardwareMap.get(Servo.class, "tester");
                outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");

                NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

                while(true){
                    intakeMotorPower = intakeControlMotor.PIDControl(0, intakeMotor.getCurrentPosition());
                    outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
                    outakeMotorPower *= PIDincrement;

                    intakeMotor.setPower(intakeMotorPower);
                    outakeRightMotor.setPower(outakeMotorPower);
                    outakeLeftMotor.setPower(outakeMotorPower);
                    intakeSpinMotor.setPower(intakeMotorPickUpPower);


                    //Set servo Positions
                    intakeRotateServo.setPosition((intakeRotateServoPosition+gravityAdder) / 360);
                    outakeArmServo.setPosition(outakeArmServoPosition / 360);
                    outakeSampleServo.setPosition(outakeSampleServoPosition / 360);
                    telemetry.addData("frontLeftPowerCat",frontLeftPowerCat);
                    telemetry.addData("backLeftPowerCat",backLeftPowerCat);
                    telemetry.addData("frontRightPowerCat",frontRightPowerCat);
                    telemetry.addData("backRightPowerCat",backRightPowerCat);

                    telemetry.addLine("This is Motor "+Thread.currentThread().getId());
                    updateTelemetry(telemetry);


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
    }
}