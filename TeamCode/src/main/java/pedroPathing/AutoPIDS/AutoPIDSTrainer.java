package pedroPathing.AutoPIDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name = "AutoPIDSTrainer", group = "Tools")
@com.acmerobotics.dashboard.config.Config
public class AutoPIDSTrainer extends LinearOpMode {

    ///-------------------------------------\\\
    /// PRESET VALUES MUST BE GIVEN BY HAND \\\
    ///-------------------------------------\\\

    public static double stillThresholdTicks = 2;
    public static long stillTimeoutMs = 20;
    public static double settleErrorThreshold = 10;
    public static int maxOscillationCycles = 400;
    public static double completionTimePercent = 0.85;
    public static boolean wasPressedBegging = false;
    public static boolean canAutoTrain = false;
    public static boolean hasNotFullPowered = true;
    public static boolean emergencyReturned = false;
    public static boolean autoTrainingFailed = false;
    public static boolean stopTelemetryUpdate = false;
    public static boolean wasBPressed = false;
    public static double gravityCompensation = 0;


    DcMotor testMotor;
    double maxDistanceTicks = 0;
    long maxSpeedTimeMs = 0;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    String pidFilename = "/sdcard/FIRST/pid_values.txt";

    @Override
    public void runOpMode() {
        //reset static stuff in init
        wasPressedBegging = false;
        canAutoTrain = false;
        hasNotFullPowered = true;
        emergencyReturned = false;
        autoTrainingFailed = false;
        stopTelemetryUpdate = false;
        wasBPressed = false;
        gravityCompensation = 0; //got bored of gravity not yed added

        testMotor = hardwareMap.get(DcMotor.class, "motor0");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //multiple telemetry
        dashboard = FtcDashboard.getInstance();
        MultipleTelemetry telemetry =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
        telemetry.update();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while(opModeIsActive()){
            if(gamepad1.a) wasPressedBegging = true;
            if(!gamepad1.a && wasPressedBegging && hasNotFullPowered){

                fullPowerRun();

                canAutoTrain = true;
                wasPressedBegging = false;
                hasNotFullPowered = false;
            }



            if (!gamepad1.a && canAutoTrain && wasPressedBegging) {

                runOscillationAutotune();

                canAutoTrain = false;
                wasPressedBegging = false;
            }

            telemetry.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
            telemetry.addLine("autoTrainingFailed");
            telemetry.addLine("waiting for button press to begin");
            if(emergencyReturned) {
                telemetry.addLine("EMERGENCY OCCURRED !!!");
                telemetry.addLine("EMERGENCY OCCURRED !!!");
                telemetry.addLine("EMERGENCY OCCURRED !!!");
                testMotor.setPower(0);
                testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if(autoTrainingFailed) telemetry.addLine("auto Training Failed ( VALUES = 0 ) ");

            if(!stopTelemetryUpdate) telemetry.update();

        }


    }

    void fullPowerRun() {
        //presets
        testMotor.setPower(1.0);
        long startTime = System.currentTimeMillis();
        long lastMoveTime = startTime;
        int lastPosition = 0;

        //will run while active and will break if it stands still
        while (opModeIsActive()) {
            int currentPosition = testMotor.getCurrentPosition();
            long currentTime = System.currentTimeMillis();

            if (Math.abs(currentPosition - lastPosition) > stillThresholdTicks) {
                lastMoveTime = currentTime;
                lastPosition = currentPosition;
            }

            if ((currentTime - lastMoveTime) > stillTimeoutMs) break;
            if(startTime + 10000 < System.currentTimeMillis()){
                emergencyReturned = true;
                break;
            }  //ran for 10 seconds

            telemetry.addData("Max Power Running",true);
            telemetry.addData("Curent Motor Position",testMotor.getCurrentPosition());
            telemetry.update();
        }

        //outputs are public static
        testMotor.setPower(0);
        maxDistanceTicks = Math.abs(testMotor.getCurrentPosition());
        maxSpeedTimeMs = System.currentTimeMillis() - startTime;

    }

    void runOscillationAutotune() {
        /// used values in auto tuning THIS CODE ISNT OPTIMIZED AND ISNT MEANT TO BE
        double p = 0.01;
        double step = 0.005;
        double ku = 0;
        double tu = 0;

        long lastPeakTime = 0;

        int peakCount = 0;
        int lastSign = 1;
        int oscCycles = 0;

        double target = maxDistanceTicks / 2.0;
        if(target == 0){
            emergencyReturned = true;
            return;
        }

        for (int trial = 0; trial < 20 && opModeIsActive(); trial++) {
            // motor checks ig just making sure
            //testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lastPeakTime = 0;
            peakCount = 0;
            oscCycles = 0;

            long startTime = System.currentTimeMillis();
            boolean stopConditionMet = false;

            while (opModeIsActive()) {
                //constantly doing stuff in a while

                int pos = testMotor.getCurrentPosition();
                double error = target - pos;
                int currentSign = (int) Math.signum(error);

                //direction checks
                if (currentSign != lastSign && lastSign != 0) {
                    long now = System.currentTimeMillis();
                    if (lastPeakTime != 0) {
                        tu = (now - lastPeakTime) / 1000.0;
                        peakCount++;
                    }
                    lastPeakTime = now;
                }

                lastSign = currentSign;

                double output = p * error;
                output = Math.max(-1.0, Math.min(1.0, output)); // just a fancy compatible math clamp
                testMotor.setPower(output);

                long elapsed = System.currentTimeMillis() - startTime;
                boolean withinTime = elapsed >= (completionTimePercent * maxSpeedTimeMs);
                boolean settled =
                        Math.abs(pos - maxDistanceTicks / 4) < settleErrorThreshold ||
                                Math.abs(pos - maxDistanceTicks / 2) < settleErrorThreshold ||
                                Math.abs(pos - 3 * maxDistanceTicks / 4) < settleErrorThreshold ||
                                Math.abs(pos - maxDistanceTicks) < settleErrorThreshold;

                if (withinTime && settled) {
                    stopConditionMet = true;
                    break;
                }

                if (++oscCycles > maxOscillationCycles) break;

                sendDashboard(pos, target, error, output, p); //dashboard graphs!
                idle(); //not sure if i actually need this but idk
            }

            testMotor.setPower(0);

            if (peakCount >= 3 || stopConditionMet) {
                ku = p;
                break;
            }

            p += step;
        }

        if (ku == 0 || tu == 0) {
            autoTrainingFailed = true;
            return;
        }

        double kp = 0.6 * ku;
        double ki = 2 * kp / tu; /// I personally strongly recommend NOT using the I term but never the less it is trained
        double kd = kp * tu / 8;

        savePIDValues(kp, ki, kd);

        telemetry.addLine("PID Tune Complete");
        telemetry.addData("Ku", ku);
        telemetry.addData("Tu", tu);
        telemetry.addData("Kp", kp);
        telemetry.addData("Ki", ki);
        telemetry.addData("Kd", kd);
        stopTelemetryUpdate = true;
        telemetry.update();
    }

    void sendDashboard(double position, double target, double error, double output, double p) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", position);
        packet.put("Target", target);
        packet.put("Error", error);
        packet.put("Output", output);
        packet.put("P", p);
        dashboard.sendTelemetryPacket(packet);
    }

    void savePIDValues(double kp, double ki, double kd) {
        //file creation for value storing
        File pidFile = new File(pidFilename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(pidFile,true))) {
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd hh:mm:ss"); // date and time might crash
            writer.write(sdf.format(new Date()));
            writer.newLine();
            writer.write("Kp=" + kp + "\n");
            writer.write("Ki=" + ki + "\n");
            writer.write("Kd=" + kd + "\n");
        } catch (IOException e) {
            telemetry.addLine("Failed to write PID values to file");
            stopTelemetryUpdate = true;
            telemetry.update();
        }
    }
}
