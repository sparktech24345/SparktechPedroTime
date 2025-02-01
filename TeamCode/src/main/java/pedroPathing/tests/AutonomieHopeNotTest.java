package pedroPathing.tests;

import static pedroPathing.PositionStorage.*;
import static pedroPathing.PositionStorage.intakeTargetPos;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import pedroPathing.ControlMotor;

@Autonomous(group = "drive")
@Disabled
public class AutonomieHopeNotTest extends LinearOpMode {
    ExecutorService executorService = Executors.newFixedThreadPool(2);
    final static boolean SLOW = true;
    final float MEEP_MOD = 0.002727f;
    SparkFunOTOS otos;
    private IMU imu = null;
    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;
    SparkFunOTOS myOtos;
    final float[] hsvValues = new float[3];
    NormalizedColorSensor colorSensor;
    long timer;
    long timer1;

    int a= 0;


    @Override
    public void runOpMode() throws InterruptedException {


        //declare servos

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

        //MultiThread
        executorService.execute(new Runnable() {
            @Override
            public void run() {
                double intakeMotorPower = 0;
                double outakeMotorPower = 0;
                DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
                DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
                DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
                DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
                DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
                DcMotor intakeSpinMotor = hardwareMap.dcMotor.get("intakespin");
                DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
                DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);//*/
                outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");


                while(opModeIsActive()){
                    intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeActualZero+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
                    outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos-outakeTargetPosAdder, outakeLeftMotor.getCurrentPosition());
                    outakeMotorPower *= PIDincrement;

                    frontLeftMotor.setPower(frontLeftPowerCat);
                    backLeftMotor.setPower(backLeftPowerCat);
                    frontRightMotor.setPower(frontRightPowerCat);
                    backRightMotor.setPower(backRightPowerCat);

                    intakeMotor.setPower(intakeMotorPower);
                    outakeRightMotor.setPower(outakeMotorPower);
                    outakeLeftMotor.setPower(outakeMotorPower);//*/
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
//*/
            }
        });




        //RUN AUTO
        timer = System.currentTimeMillis();

        while(timer+600 > System.currentTimeMillis() && opModeIsActive() ){
            OuttakeStateSpecimen();
        }

        timer = System.currentTimeMillis();

        while(timer+800 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat=0.6;
            backLeftPowerCat= 0.6;
            frontRightPowerCat=0.6;
            backRightPowerCat=0.6;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);

        timer = System.currentTimeMillis();

        while(timer + 100> System.currentTimeMillis()  && opModeIsActive())
            a = 1;

        timer = System.currentTimeMillis();

        while(timer+800 > System.currentTimeMillis()  && opModeIsActive()){
            if (timer + 100 < System.currentTimeMillis())
                outakeTargetPos = -1900;
            if (timer + 600 < System.currentTimeMillis())
                outakeSampleServoPosition = servoextended;
        }
        timer = System.currentTimeMillis();

        while(timer+ 200 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat = 0.0001;
            backLeftPowerCat=-1;
            frontRightPowerCat=-1;
            backRightPowerCat=0.0001;
        }
        timer = System.currentTimeMillis();
        //rotate
        OuttakeStateSamplePickUp();
        while(timer+ 550 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat = - 0.6;
            backLeftPowerCat= - 0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= 0.6;
        }
        timer = System.currentTimeMillis();
        //go forward(back)
        while(timer+ 450 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat = - 0.6;
            backLeftPowerCat= - 0.6;
            frontRightPowerCat= -0.6;
            backRightPowerCat= -0.6;
        }
        timer = System.currentTimeMillis();
        //adjust
        while(timer+ 100 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.6;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);

        timer = System.currentTimeMillis();

        //pickup
        while(timer + 300> System.currentTimeMillis()  && opModeIsActive())
            IntakeExtended();

        timer = System.currentTimeMillis();
        shouldTransfer = false;
        while(timer+ 8000 > System.currentTimeMillis()  && opModeIsActive() && !shouldTransfer){
            intakeTargetPos = 350;
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (colors.red >= 0.005 || colors.blue >= 0.005) {
                    shouldTransfer = true;
            }
        }

        timer = System.currentTimeMillis();
        shouldTransfer = false;
        //rotate to place
        while(timer+ 600 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.4;
            backLeftPowerCat=  0.4;
            frontRightPowerCat= - 0.4;
            backRightPowerCat= - 0.4;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        timer = System.currentTimeMillis();
        //place 1
        while(timer+ 200 > System.currentTimeMillis()  && opModeIsActive())
            IntakeOutput();
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        timer = System.currentTimeMillis();
        shouldTransfer = false;
        //rotate to pick up 2
        while(timer+ 800 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat = - 0.4;
            backLeftPowerCat= - 0.4;
            frontRightPowerCat=  0.4;
            backRightPowerCat=  0.4;
        }

        timer = System.currentTimeMillis();
        while(timer+ 800 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  -0.6;
            backLeftPowerCat=  0.6;
            frontRightPowerCat= 0.6;
            backRightPowerCat= -0.6;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);





        timer = System.currentTimeMillis();
        shouldTransfer = false;
        while(timer+ 8000 > System.currentTimeMillis()  && opModeIsActive() && !shouldTransfer){
            intakeTargetPos = 50;
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (colors.red >= 0.005 || colors.blue >= 0.005) {
                shouldTransfer = true;
            }
        }

        while(timer+ 600 > System.currentTimeMillis()  && opModeIsActive()){
            frontLeftPowerCat =  0.4;
            backLeftPowerCat=  0.4;
            frontRightPowerCat= - 0.4;
            backRightPowerCat= - 0.4;
            intakeTargetPos = 350;
        }
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);
        timer = System.currentTimeMillis();

        while(timer+ 200 > System.currentTimeMillis()  && opModeIsActive())
            IntakeOutput();
        frontLeftPowerCat=(0);
        backLeftPowerCat=(0);
        frontRightPowerCat=(0);
        backRightPowerCat=(0);




        if (isStopRequested()) return;

    }


    public void OuttakeStateSpecimen(){
        telemetry.addData("OuttakeStateSpecimen",true);
        isOuttakeStateSpecimen = true;
        wasOuttakeStateSpecimen= true;
        outakeSampleServoPosition= outakeSampleRetracted;
        outakeArmServoPosition = 345;
        if (intakeMotorPickUpPower != 1)
            intakeMotorPickUpPower =0;
        outakeTargetPos = -outakeTargetPosAdder-1400;
    }
    public void OuttakeStateSamplePickUp(){ // ONe. . ..OuttakeStateSpecimen
        telemetry.addData("OuttakeStateSamplePickUp",true);
        isOuttakeStateSamplePickUp = true;
        outakeArmServoPosition = 0;
        outakeTargetPos =0;
        outakeSampleServoPosition = servoextended;
        if (intakeMotorPickUpPower != 1)
            intakeMotorPickUpPower = 0;
    }
    public void IntakeExtended(){
        isIntakeStateExtended = true;
        wasIntakeStateExtended = true;
        wasActiveintake = true;
        wasActivePastActiveIntake = true;
        wasBambuExtended = true;
        intakeRotateServoPosition = IntakeServoColectPos;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = 0.7;
    }
    public void IntakeOutput(){
        isIntakeStateExtended = true;
        wasIntakeStateExtended = true;
        wasActiveintake = true;
        wasActivePastActiveIntake = true;
        wasBambuExtended = true;
        intakeRotateServoPosition = IntakeServoColectPos;
        if(gravityAdder==0)
            gravityAdder = 7;
        intakeMotorPickUpPower = -0.7;
    }
}
