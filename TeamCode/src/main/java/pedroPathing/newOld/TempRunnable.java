package pedroPathing.newOld;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static pedroPathing.newOld.PositionStorage.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class TempRunnable implements Runnable {

    volatile boolean keepRunning = true;
    double intakeMotorPower;
    double outakeMotorPower;

    ControlMotor intakeControlMotor;
    ControlMotor outakeControlMotor;


    public TempRunnable() {}
    public void stopRunning() {
        this.keepRunning=false;
    }
    private boolean DoOnce = true;

    @Override
    public void run() {
        try {
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
            DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
            DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
            DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
            intakeControlMotor = new ControlMotor();
            outakeControlMotor = new ControlMotor();
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            outakeLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            DoOnce = false;
        while (keepRunning) {
            frontLeftMotor.setPower(frontLeftPowerCat);
            backLeftMotor.setPower(backLeftPowerCat);
            frontRightMotor.setPower(frontRightPowerCat);
            backRightMotor.setPower(backRightPowerCat);//*/
            intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos, intakeMotor.getCurrentPosition());
            outakeMotorPower = outakeControlMotor.PIDControlUppy(outakeTargetPos, outakeRightMotor.getCurrentPosition());
            intakeMotor.setPower(intakeMotorPower);
            outakeRightMotor.setPower(outakeMotorPower);
            outakeLeftMotor.setPower(outakeMotorPower);
            telemetry.addData("Hello World", true);
            telemetry.update();
            Thread.sleep(15);
        }
        } catch (InterruptedException ignored) {
        }
    }
}

