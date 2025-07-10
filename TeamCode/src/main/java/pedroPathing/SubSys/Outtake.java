package pedroPathing.SubSys;

import static pedroPathing.OrganizedPositionStorage.outtakeClawServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeExtendMotorTargetPos;
import static pedroPathing.OrganizedPositionStorage.outtakePivotServoPos;
import static pedroPathing.OrganizedPositionStorage.outtakeTargetPosAdder;
import static pedroPathing.OrganizedPositionStorage.tempOuttakeAPosition;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.PIDStorageAndUse.ControlMotor;


@com.acmerobotics.dashboard.config.Config
public class Outtake {
    HardwareMap mapy;
    MultipleTelemetry teller;
    public DcMotor outakeLeftMotor;
    public DcMotor outakeRightMotor;
    public Servo outakeArmServo;
    public Servo outakeSampleServo;
    ControlMotor outakeControlMotor = new ControlMotor();

    public Outtake(HardwareMap maps, MultipleTelemetry telly){
        this.mapy = maps;
        this.teller = telly;


        this.outakeLeftMotor = mapy.dcMotor.get("outakeleftmotor");
        this.outakeRightMotor = mapy.dcMotor.get("outakerightmotor");
        this.outakeArmServo = mapy.get(Servo.class, "outakeArmServo");
        this.outakeSampleServo = mapy.get(Servo.class, "outakeSampleServo");

        outakeRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void outtakeGivePower(){
        double outtakeExtendMotorPow = -outakeControlMotor.PIDControlUppy(-outtakeExtendMotorTargetPos-outtakeTargetPosAdder, -outakeLeftMotor.getCurrentPosition());

        outakeRightMotor.setPower(outtakeExtendMotorPow);
        outakeLeftMotor.setPower(outtakeExtendMotorPow);

        outakeArmServo.setPosition(outtakePivotServoPos / 328);
        outakeSampleServo.setPosition(outtakeClawServoPos / 360);

        tempOuttakeAPosition = outakeLeftMotor.getCurrentPosition();
    }

}
