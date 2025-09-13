package pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

import pedroPathing.constants.FConstantsForPinpoint;
import pedroPathing.constants.LConstantsForPinpoint;
import pedroPathing.constants.LConstantsForPinpointWIthREversedForward;

@Config
@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode{
    public static double deltaX=0;
    public static double deltaY=0;
    public double calculateHeadingAdjustment(Pose robotPose, double targetX, double targetY) {
        double x = robotPose.getX();
        double y = - robotPose.getY();


        // Vector from robot to target
        double dx = targetX - x;
        double dy = targetY - y;

        // Angle from robot position to target point
        double targetAngle = Math.atan2(dy, dx);
        double targetDegrees = Math.toDegrees(targetAngle); //is from 0to eather 180 or -180, same as robot heading Maxed

        //robot heading stuff  //its negative as its inversed from what youd want ( right +, left -)
        double robotAngleNormal = Math.toDegrees(robotPose.getHeading()) -90;
        //no negatives
        if(robotAngleNormal <0) robotAngleNormal = 360 + robotAngleNormal;

        //maxing out the angle on a 180 interval with + and - for easy servo math
        double robotAnglesMaxed;
        if(robotAngleNormal > 180) robotAnglesMaxed = robotAngleNormal - 360; //needs to be negative, and start from the same 0
        else robotAnglesMaxed = robotAngleNormal;

        // so basicly you will have from 0 to 180 then imidiatly -180 to 0 ( same 0 )
        // so the robot 50 degrees to right is +50 and 50 degrees to the left is -50

        multipleTelemetry.addData("corrected head",robotAnglesMaxed);
        multipleTelemetry.addData("calculated angle ",targetDegrees);

        return targetDegrees   + robotAnglesMaxed;
    }
    public double addaptForServoTurret(double degrees, double maxDegreesLeft, double maxDegreesRight){
        //the 0 of the servo is at its maximum left point, thus i must normalize it by using negatives for left,
        //while in reality middle is actually maxleft and max right would be its complete maximum position thus both maxes combined
        //thats why for the left part i just invert the number and for the right part i add the maximum left
        if(degrees <= 0){
            // for the left part is how many degrees is want from center ( max let for servo ) to the left ( 0 )
            degrees = maxDegreesLeft + degrees;
            return degrees;
            //will be negative if turret wants to go more then max
        }
        else if (degrees > 0) {
            degrees += maxDegreesLeft;
            return degrees;
            //will be over both max if turret wants to go more then right max
        }
        else{
            //how did we get here, go to middle point
            return maxDegreesLeft;
        }
    }

    public double clampAngleForServo(double degrees,double maxServoDegree, MultipleTelemetry multipletelemetry){
        if(degrees < 0) multipletelemetry.addData("too much to the left by", degrees);
        if(degrees > maxServoDegree) multipletelemetry.addData("too much to the right by", maxServoDegree-degrees);
        return Math.max(0, Math.min(maxServoDegree, degrees));
    }

    public static double targetX = 0;
    public static double targetY = 25;

    double turretServoPos=90;
    Follower follower;
    MultipleTelemetry multipleTelemetry;
    double degreesToMove = 0;
    double degreesForServo = 0;
    double safeDegreesForServo = 0;

    /* variabile pt codul meu */

    private final Pose follower_pose = new Pose(0, 0, Math.toRadians(90));
    private Pose actual_pose;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo = hardwareMap.get(Servo.class, "turretServo");
        Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpointWIthREversedForward.class);
        follower = new Follower(hardwareMap, FConstantsForPinpoint.class, LConstantsForPinpointWIthREversedForward.class);
        follower.setStartingPose(follower_pose);
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();

            multipleTelemetry.addData(" Turret wanna be pos",degreesToMove);
            multipleTelemetry.addData("servo actual pose",turretServo.getPosition());
            multipleTelemetry.addData("servo actual pose * 300",turretServo.getPosition()*300);
            multipleTelemetry.addData("x",follower.getPose().getX());
            multipleTelemetry.addData("y",follower.getPose().getY());
            multipleTelemetry.addData("head",Math.toDegrees(follower.getPose().getHeading()));
            multipleTelemetry.addData("targeting", Math.toDegrees(Math.atan2(deltaY, deltaX)));
            follower.drawOnDashBoard();

            degreesToMove = calculateHeadingAdjustment(follower.getPose(),8,0);
            degreesForServo = addaptForServoTurret(degreesToMove,150,150);
            safeDegreesForServo = clampAngleForServo(degreesForServo,300,multipleTelemetry);

            multipleTelemetry.update();
            turretServo.setPosition(safeDegreesForServo/300);
        }
    }
}