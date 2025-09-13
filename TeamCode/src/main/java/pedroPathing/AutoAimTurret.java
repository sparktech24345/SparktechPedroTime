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

@Config
@TeleOp(name = "AutoAimTurret", group = "LinearOpMode")
public class AutoAimTurret extends LinearOpMode{
    public double calculateHeadingAdjustment(Pose robotPose, double targetX, double targetY) {
        // Vector from robot to target
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // Angle from robot position to target point
        double targetAngle = Math.atan2(dy, dx);

        // Difference between target direction and robot heading
        double angleDiff = Math.toDegrees(normalizeAngle(targetAngle - robotPose.getHeading()));

        return angleDiff;
    }

    /**
     * Normalizes the angle to the range [-PI, PI]
     */
    public double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
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
        multipletelemetry.update();
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

    private Pose follower_pose = new Pose(0, 0, 0);
    private Pose actual_pose;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretServo = hardwareMap.get(Servo.class, "turretServo");
        Constants.setConstants(FConstantsForPinpoint.class, LConstantsForPinpoint.class);
        follower = new Follower(hardwareMap,FConstantsForPinpoint.class, LConstantsForPinpoint.class);

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();

            multipleTelemetry.addData(" Turret wanna be pos",degreesToMove);
            multipleTelemetry.addData("servo actual pose",turretServo.getPosition());
            multipleTelemetry.addData("servo actual pose * 360",turretServo.getPosition()*360);
            multipleTelemetry.update();
            follower.drawOnDashBoard();

            degreesToMove = calculateHeadingAdjustment(follower.getPose(),0,5);
            degreesForServo = addaptForServoTurret(degreesToMove,150,150);
            safeDegreesForServo = clampAngleForServo(degreesForServo,300,multipleTelemetry);

            turretServo.setPosition(safeDegreesForServo/300);
        }
    }
}