package pedroPathing.AutoPIDTuner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoPIDTuner {
    //---------------MOTOR METHOD DECLARATION----------------------\\

    DcMotor mainMotor,auxMotor1;
    void setMotors(){
        this.mainMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        this.auxMotor1 = hardwareMap.dcMotor.get("outakerightmotor");
        this.auxMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    void motorsSetPower(double power) {
        mainMotor.setPower(power);
        auxMotor1.setPower(power);
    }


    //-------------SERVO STUFF ( OPTIONAL )-----------------\\
    void setServos(){
        Servo servo1 = hardwareMap.get(Servo.class, "outakeArmServo");
        servo1.setPosition(90 / 328);
        Servo servo2 = hardwareMap.get(Servo.class, "intakeRotateServo");
        servo2.setPosition((30) / 228);
    }




    HardwareMap hardwareMap;
    public AutoPIDTuner(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        setMotors();
    }



}
