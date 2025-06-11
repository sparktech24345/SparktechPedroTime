package pedroPathing.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.AutoPIDS.ControlMotor;

@Autonomous(group = "drive")
@Disabled
public class AutonomieTest extends LinearOpMode {
    final static boolean SLOW = true;
    final float MEEP_MOD = 0.002727f;
    SparkFunOTOS otos;
    double intakeRotateServoPosition = 70;
    private IMU imu = null;
    double outakeArmServoPosition = 0;
    double outakeSampleServoPosition = 10;
    double intakeTargetPos = 0;
    double outakeTargetPos =0;
    double outakeTargetPosAdder =0;
    long startingTimer;
    long startingTimer2;
    long colortimer;
    long outPutTimer;
    boolean wasActive = false;
    boolean wasActive2 = false;
    double outakeRotateServoPosition = 60;
    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    final float[] hsvValues = new float[3];
    NormalizedColorSensor colorSensor;
    double intakeServoPower = 0;
    boolean isIntakeStateExtended = false;
    boolean isIntakeStateRectracted = false;
    boolean isOuttakeStateStandbyWithSample = false;
    boolean isOuttakeStateSamplePickUp = false;
    boolean isOuttakeStateBascket = false;
    boolean isOuttakeStateSpecimen = false;
    boolean wasIntakeStateExtended = false;
    boolean wasActiveintake =false;
    boolean wasOuttakeStateBascket = false;
    boolean wasOuttakeStateSpecimen = false;
    boolean wascolor = false;
    double PIDincrement =1;
    int servoextended = 90;
    long timer;
    long timer1;
    double outakeMotorPower = 0;



    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");

        //declare servos
        CRServo servo1 = hardwareMap.get(CRServo.class, "rightservo");
        CRServo servo2 = hardwareMap.get(CRServo.class, "leftservo");
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        Servo outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        Servo outakeRotateServo = hardwareMap.get(Servo.class, "outakeRotateServo");
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        intakeControlMotor = new ControlMotor();
        outakeControlMotor = new ControlMotor();

        outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        otos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");
        Config.configureOtos(telemetry, otos);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        waitForStart();


        otos.begin();
        double otosh = otos.getPosition().h;
        float angle = (float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (otosh < 0) otosh = otosh + 360;
        imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        imu.getRobotOrientationAsQuaternion();
        imu.getRobotYawPitchRollAngles();


        if (isStopRequested()) return;
    }


    public void OuttakeStateSpecimen(){
        telemetry.addData("OuttakeStateSpecimen",true);
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        outakeSampleServoPosition=0+3;
        outakeArmServoPosition = 180;
        outakeRotateServoPosition=0+90;
        if (intakeServoPower != 1)
            intakeServoPower =0;
        outakeTargetPos = -20-outakeTargetPosAdder-340;
    }
    public void OuttakeStateSamplePickUp(){ // ONe. . ..OuttakeStateSpecimen
        telemetry.addData("OuttakeStateSamplePickUp",true);
        isOuttakeStateSamplePickUp = true;
        outakeArmServoPosition = 0;
        outakeRotateServoPosition = 60 + 90;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
        if (intakeServoPower != 1)
            intakeServoPower = 0;
    }
}
