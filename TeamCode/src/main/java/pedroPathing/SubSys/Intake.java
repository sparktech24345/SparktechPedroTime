package pedroPathing.SubSys;

import static pedroPathing.OrganizedPositionStorage.*;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.ControlMotor;

@com.acmerobotics.dashboard.config.Config
public class Intake {

    HardwareMap mapy;
    MultipleTelemetry teller;
    public DcMotor intakeMotor;
    public DcMotor intakeSpinMotor;
    public Servo intakeRotateServo;
    ControlMotor intakeControlMotor = new ControlMotor();

    public Intake(HardwareMap maps, MultipleTelemetry telly){
        this.mapy = maps;
        this.teller = telly;


        this.intakeMotor = mapy.dcMotor.get("intakemotor");
        this.intakeSpinMotor = mapy.dcMotor.get("intakespin");
        this.intakeRotateServo = mapy.get(Servo.class, "intakeRotateServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeGivePower(){
        double intakeExtendMotorPow = intakeControlMotor.PIDControl(intakeExtendMotorTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());

        intakeMotor.setPower(intakeExtendMotorPow);
        intakeSpinMotor.setPower(intakeSpinMotorPow);

        intakeRotateServo.setPosition((intakePivotServoPos-intakeGravitySubtractor) / 228);
    }

}
