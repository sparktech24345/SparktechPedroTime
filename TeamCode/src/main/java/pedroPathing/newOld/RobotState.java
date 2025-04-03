package pedroPathing.newOld;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotState {
    private static RobotState robotState=null;
    public boolean isValid;

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;
    DcMotor outakeLeftMotor;
    DcMotor outakeRightMotor;
    //declare servos
    CRServo servo1;
    CRServo servo2;
    Servo intakeRotateServo;
    Servo outakeArmServo;
    Servo outakeSampleServo;
    Servo outakeRotateServo;
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection;
    RevHubOrientationOnRobot orientationOnRobot;
    IMU imu = null;

    private RobotState() {
        RobotState.robotState=this;
        this.isValid=true;
    }

    public static RobotState getRobotState() {
        return robotState;
    }
    public static RobotState init(HardwareMap hardwareMap) {
        if(robotState!=null) {
            robotState.isValid=false;
         }

        robotState = new RobotState();
        robotState.frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        robotState.backLeftMotor = hardwareMap.dcMotor.get("backleft");
        robotState.frontRightMotor = hardwareMap.dcMotor.get("frontright");
        robotState.backRightMotor = hardwareMap.dcMotor.get("backright");
        robotState.intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        robotState.outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        robotState.outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        robotState.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotState.outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotState.myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFunSensor");
        //declare servos
        robotState.servo1 = hardwareMap.get(CRServo.class, "rightservo");
        robotState.servo2 = hardwareMap.get(CRServo.class, "leftservo");
        robotState.intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        robotState.outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        robotState.outakeSampleServo = hardwareMap.get(Servo.class, "outakeSampleServo");
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        robotState.intakeControlMotor = new ControlMotor();
        robotState.outakeControlMotor = new ControlMotor();

        robotState.colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");

        robotState.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotState.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotState.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotState.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotState.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotState.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //TODO investigate RUN_WITH_ENCODER


        robotState.logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        robotState.usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        robotState.orientationOnRobot = new RevHubOrientationOnRobot(robotState.logoDirection, robotState.usbDirection);

        robotState.imu = hardwareMap.get(IMU.class, "imu");
        robotState.imu.initialize(new IMU.Parameters(robotState.orientationOnRobot));

        robotState.imu.resetYaw();
        return robotState;
    }
}
