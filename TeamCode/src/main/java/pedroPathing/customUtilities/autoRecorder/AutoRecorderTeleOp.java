package pedroPathing.customUtilities.autoRecorder;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.localization.Pose;

import java.io.File;
import java.util.ArrayList;

@TeleOp(name="Autonomy Recorder", group="Linear OpMode")
public class AutoRecorderTeleOp extends LinearOpMode {
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @SuppressLint("SdCardPath")
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.setMsTransmissionInterval(50);

        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);

        Telemetry tel = new MultipleTelemetry(this.telemetry, dashboardTelemetry);

        waitForStart();

        if (isStopRequested()) return;

        boolean wasA1Pressed=false;
        boolean wasB1Pressed=false;
        boolean wasY1Pressed=false;
        PoseData lastPose = new PoseData(0,0,Math.toRadians(0));

        int index = 0;
        ArrayList<PoseSequence> paths = new ArrayList<>();
        paths.add(new PoseSequence());
        PoseSequence poseSequence = paths.get(index);
        poseSequence.addPose(new PoseData(startPose.getX(), startPose.getX(), startPose.getHeading()));

        while (opModeIsActive())
        {
            if(gamepad1.a){
                wasA1Pressed = true;
            }if(!gamepad1.a && wasA1Pressed){
                lastPose = new PoseData(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

                poseSequence = paths.get(index);
                poseSequence.addPose(lastPose);

                paths.add(new PoseSequence());

                index++;

                wasA1Pressed = false;
            }
            if(gamepad1.b){
                wasB1Pressed = true;
            }if(!gamepad1.b && wasB1Pressed){
                lastPose = new PoseData(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

                poseSequence = paths.get(index);
                poseSequence.addPose(lastPose);

                //paths.add(new PoseSequence());

                //index++;

                wasB1Pressed = false;
            }
            tel.addLine("Current position: ");
            tel.addData("x cur", follower.getPose().getX());
            tel.addData("y cur", follower.getPose().getY());
            tel.addData("heading cur", Math.toDegrees(follower.getPose().getHeading()));
            tel.addLine("Last recorded position: ");
            tel.addData("x", lastPose.getX());
            tel.addData("y", lastPose.getY());
            tel.addData("heading", lastPose.getHeading());
            follower.update();
            tel.update();

            ObjectMapper mapper = new ObjectMapper();

            if(gamepad1.y){
                wasY1Pressed = true;
            }if(!gamepad1.y && wasY1Pressed){
                wasY1Pressed = false;
                try {
                    mapper.writerWithDefaultPrettyPrinter() // pretty-print
                            .writeValue(new File("/sdcard/FIRST/path.json"), paths);


                        tel.addLine("File generated");
                        tel.update();
                        requestOpModeStop();

                } catch (Exception e) {

                        tel.addLine("Error in generating");
                        tel.update();
                        requestOpModeStop();
                    
                    //e.printStackTrace();
                }

            }



        }
    }
}