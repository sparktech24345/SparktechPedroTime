package Experimental.Modules;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Experimental.HelperClasses.OpMode;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

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
    private Follower follower;

    public void init() {
        if (currentOpMode == OpMode.TeleOP) {
        RFDrive = hardwareMap.get(DcMotor.class, "frontright");
        LFDrive = hardwareMap.get(DcMotor.class, "frontleft");
        RBDrive = hardwareMap.get(DcMotor.class, "backright");
        LBDrive = hardwareMap.get(DcMotor.class, "backleft");

        LFDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            follower.setStartingPose(startPos);
            follower.update();
        }
    }

    public void start() {}

    public void loop(boolean IsRunningAuto) {
        if (!IsRunningAuto) {


        }
    }

    public void stop() {}
}
