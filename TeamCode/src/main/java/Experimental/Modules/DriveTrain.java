package Experimental.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
* 
*  !! IMPORTANT !!
* 
*  THE DRIVETRAIN CLASS USES THE FOLLOWING ABBREVIATIONS:
*  RF - Right Front
*  LF - Left Front
*  RB - Right Back
*  LB - Left Back
* 
* */
public class DriveTrain extends BaseModule {

    private DcMotor RFDrive;
    private DcMotor LFDrive;
    private DcMotor RBDrive;
    private DcMotor LBDrive;

    public void init() {
        RFDrive = hardwareMap.get(DcMotor.class, "frontright");
        LFDrive = hardwareMap.get(DcMotor.class, "frontleft");
        RBDrive = hardwareMap.get(DcMotor.class, "backright");
        LBDrive = hardwareMap.get(DcMotor.class, "backleft");
        
        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void start() {}

    public void loop(boolean IsRunningAuto) {
        if (!IsRunningAuto) {


        }
    }

    public void stop() {}
}
