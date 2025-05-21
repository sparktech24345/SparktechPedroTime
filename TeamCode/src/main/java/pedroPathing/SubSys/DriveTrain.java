package pedroPathing.SubSys;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.newOld.Toggle;

@com.acmerobotics.dashboard.config.Config
public class DriveTrain {
    public static double driveTrainMuliplier = 0.8;
    public static double manualSlowDownMultiplier = 1.5;

    Gamepad gamepad;
    HardwareMap mapy;
    MultipleTelemetry teller;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    public DriveTrain(HardwareMap maps, MultipleTelemetry telly, Gamepad gamepad){
        this.mapy = maps;
        this.teller = telly;
        this.gamepad = gamepad;

        this.frontLeftMotor = mapy.dcMotor.get("frontleft");
        this.backLeftMotor = mapy.dcMotor.get("backleft");
        this.frontRightMotor = mapy.dcMotor.get("frontright");
        this.backRightMotor = mapy.dcMotor.get("backright");


        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Toggle.toggled = false;
        Toggle.toggle_var = false;
    }

    public void driveTrainGivePower(){
        double vertical = gamepad.left_stick_y;
        double horizontal = gamepad.left_stick_x;
        double pivot = -gamepad.right_stick_x;

        if(Toggle.FirsToggle(gamepad.left_trigger >= 0.4 && gamepad.right_trigger >=0.4)){
            horizontal = - horizontal;
            vertical = - vertical;
        }

        // Front / Back  +  Right / Left  +  Power

        double FRP = (pivot - vertical - horizontal);
        double BRP= (pivot - vertical + horizontal);
        double FLP = (pivot + vertical - horizontal);
        double BLP = (pivot + vertical + horizontal);


        if(Toggle.toggleButton2(gamepad.right_bumper) == 1) {
            FRP = FRP / manualSlowDownMultiplier;
            BRP = BRP / manualSlowDownMultiplier;
            FLP = FLP / manualSlowDownMultiplier;
            BLP = BLP / manualSlowDownMultiplier;
        }

        frontLeftMotor.setPower(FRP*driveTrainMuliplier);
        backLeftMotor.setPower(BRP*driveTrainMuliplier);
        frontRightMotor.setPower(FLP*driveTrainMuliplier);
        backRightMotor.setPower(BLP*driveTrainMuliplier);
    }

}
