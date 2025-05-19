package pedroPathing.SubSys;

import static pedroPathing.ClassWithStates.ColorCompare;
import static pedroPathing.ClassWithStates.currentStateOfSampleInIntake;
import static pedroPathing.ClassWithStates.currentTeam;
import static pedroPathing.ClassWithStates.initStates;
import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.isYellowSampleNotGood;
import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import pedroPathing.ClassWithStates;
import pedroPathing.tests.Config;

public class OurRobot {

    Gamepad gamepad1;
    Gamepad gamepad2;
    HardwareMap mapy;
    MultipleTelemetry teller;
    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    final float[] hsvValues = new float[3];
    SparkFunOTOS myOtos;
    NormalizedColorSensor colorSensor;
    public NormalizedRGBA colors = new NormalizedRGBA();
    public SparkFunOTOS.Pose2D pos;

    public OurRobot(HardwareMap maps, MultipleTelemetry teller, Gamepad gamepady,Gamepad gamenoty){
        this.mapy = maps;
        this.teller = teller;
        this.gamepad1 = gamepady;
        this.gamepad2 = gamenoty;

        this.driveTrain = new DriveTrain(mapy,teller,gamepad1);
        this.intake = new Intake(mapy,teller);
        this.outtake = new Outtake(mapy,teller);

        this.myOtos = mapy.get(SparkFunOTOS.class, "SparkFunSensor");
        this.colorSensor = mapy.get(NormalizedColorSensor.class, "sensorColor");
    }


    public void robotSetOnlyServoPosition(){
        intake.intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
        outtake.outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outtake.outakeSampleServo.setPosition(outtakeClawServoPos / 360);
    }

    public void initRobot(){
        Config.configureOtosMultipleTel(teller, myOtos);
        initStates();
        robotSetOnlyServoPosition();
    }
    public void doConstantStuff(){
        //otos stuff
        pos = myOtos.getPosition();

        //color stuff
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        currentStateOfSampleInIntake = ColorCompare(colors,currentTeam,isYellowSampleNotGood);

        // team select
        if ((gamepad2.left_bumper && gamepad2.start) || (gamepad1.left_bumper && gamepad1.start))
            currentTeam = ClassWithStates.colorList.blue;
        if ((gamepad2.right_bumper && gamepad2.start) || (gamepad1.right_bumper && gamepad1.start))
            currentTeam = ClassWithStates.colorList.red;



        //telemetry update
        //teller.update();

    }


    public void robotSetPower(){
        outtake.outtakeGivePower();
        intake.intakeGivePower();
        driveTrain.driveTrainGivePower();
    }


}
