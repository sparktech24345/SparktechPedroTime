package pedroPathing.AutoPIDS;

import static pedroPathing.OrganizedPositionStorage.intakeGravitySubtractor;
import static pedroPathing.OrganizedPositionStorage.intakePivotServoPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    public static long stuckTimeoutMs = 2000;  // 2 seconds without motion is considered stuck
    public static double stuckThresholdTicks = 3;  // Minimum tick change to count as motion

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
    MultipleTelemetry tel;

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

        testMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        intakeRotateServo.setPosition((30) / 228);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //multiple telemetry
        dashboard = FtcDashboard.getInstance();
        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        tel.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
        tel.update();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        while(opModeIsActive()){
            if(gamepad1.a) wasPressedBegging = true;
            if(!gamepad1.a && wasPressedBegging && hasNotFullPowered){

                fullPowerRun();
                returnMotor();

                canAutoTrain = true;
                wasPressedBegging = false;
                hasNotFullPowered = false;
            }



            if (!gamepad1.a && canAutoTrain && wasPressedBegging) {

                runOscillationAutotune();

                canAutoTrain = false;
                wasPressedBegging = false;
            }

            tel.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
            tel.addLine("waiting for button press to begin");
            if(emergencyReturned) {
                tel.addLine("EMERGENCY OCCURRED !!!");
                tel.addLine("EMERGENCY OCCURRED !!!");
                tel.addLine("EMERGENCY OCCURRED !!!");
                testMotor.setPower(0);
                testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if(autoTrainingFailed) tel.addLine("auto Training Failed ( VALUES = 0 ) ");

            if(!stopTelemetryUpdate) tel.update();
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

            tel.addData("Max Power Running",true);
            tel.addData("Curent Motor Position",testMotor.getCurrentPosition());
            tel.update();
        }

        //outputs are public static
        testMotor.setPower(0);
        maxDistanceTicks = Math.abs(testMotor.getCurrentPosition());
        maxSpeedTimeMs = System.currentTimeMillis() - startTime;

    }


    void returnMotor() {
        tel.addLine("Returning motor to start position at 50% power...");
        tel.update();

        testMotor.setPower(-0.5);  // Reverse at half power
        long startTime = System.currentTimeMillis();
        long lastMoveTime = startTime;
        int lastPosition = testMotor.getCurrentPosition();

        while (opModeIsActive()) {
            int currentPosition = testMotor.getCurrentPosition();
            long currentTime = System.currentTimeMillis();

            if (Math.abs(currentPosition - lastPosition) > stillThresholdTicks) {
                lastMoveTime = currentTime;
                lastPosition = currentPosition;
            }

            // Exit if motor has been stalled
            if ((currentTime - lastMoveTime) > stillTimeoutMs) break;

            // Safety timeout
            if (currentTime - startTime > 10000) {
                emergencyReturned = true;
                break;
            }

            tel.addData("Returning Motor", true);
            tel.addData("Current Position", testMotor.getCurrentPosition());
            tel.update();
        }

        testMotor.setPower(0);
    }

    void runOscillationAutotune() {
        double p = 0.01;
        double step = 0.005;
        double minStep = 0.0001;
        double ku = 0;
        double tu = 0;

        double target = maxDistanceTicks / 2.0;
        if (target == 0) {
            emergencyReturned = true;
            return;
        }

        boolean pidCalculated = false;
        boolean increasing = true;
        double lastOscillationAmplitude = Double.MAX_VALUE;

        for (int trial = 0; trial < 20 && opModeIsActive(); trial++) {
            tel.addData("Trial", trial + 1);
            tel.addData("Current P", p);
            tel.update();

            long lastPeakTime = 0;
            int peakCount = 0;
            int lastSign = 1;
            int oscCycles = 0;

            double maxError = 0;
            double minError = Double.MAX_VALUE;

            // --- EXTEND PHASE ---
            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - startTime < 10000)) {
                int pos = testMotor.getCurrentPosition();
                double error = target - pos;

                maxError = Math.max(maxError, Math.abs(error));
                minError = Math.min(minError, Math.abs(error));

                int sign = (int) Math.signum(error);
                if (sign != lastSign && lastSign != 0) {
                    long now = System.currentTimeMillis();
                    if (lastPeakTime != 0) {
                        tu = (now - lastPeakTime) / 1000.0;
                        peakCount++;
                    }
                    lastPeakTime = now;
                }
                lastSign = sign;

                double output = p * error;
                output = Math.max(-1.0, Math.min(1.0, output));
                testMotor.setPower(output);

                oscCycles++;
                if (oscCycles > maxOscillationCycles) break;

                sendDashboard(pos, target, error, output, p);
                idle();
            }

            testMotor.setPower(0);

            // Oscillation amplitude estimate:
            double oscillationAmplitude = maxError - minError;

            // --- ANALYSIS ---
            if (!pidCalculated && peakCount >= 3 && tu > 0) {
                ku = p;
                pidCalculated = true; // flag to prevent re-calculating
            }

            // Decide whether oscillation improved or got worse
            if (oscillationAmplitude > lastOscillationAmplitude) {
                // Worse - reverse direction and reduce step
                increasing = !increasing;
                step *= 0.5;
                if (step < minStep) step = minStep;
            }

            lastOscillationAmplitude = oscillationAmplitude;

            // --- RETRACT PHASE ---
            long retractStart = System.currentTimeMillis();
            testMotor.setPower(-0.5);
            int lastPos = testMotor.getCurrentPosition();

            while (opModeIsActive() && System.currentTimeMillis() - retractStart < 5000) {
                int currPos = testMotor.getCurrentPosition();

                if (Math.abs(currPos) < stillThresholdTicks) {
                    break;
                }

                if (Math.abs(currPos - lastPos) < stillThresholdTicks) {
                    // stuck detection
                    if (System.currentTimeMillis() - retractStart > 3000) break;
                } else {
                    lastPos = currPos;
                    retractStart = System.currentTimeMillis(); // reset timeout
                }

                tel.addData("Retracting...", currPos);
                tel.update();
                idle();
            }

            testMotor.setPower(0);

            // Adjust p for next trial
            if (increasing) {
                p += step;
            } else {
                p -= step;
                if (p < 0) p = 0.001;  // avoid negative or zero p
            }
        }

        if (!pidCalculated || ku == 0 || tu == 0) {
            autoTrainingFailed = true;
            tel.addLine("Auto training failed — no stable oscillation detected.");
            tel.update();
            return;
        }

        double kp = 0.6 * ku;
        double ki = 2 * kp / tu;
        double kd = kp * tu / 8;

        savePIDValues(kp, ki, kd);

        tel.addLine("PID Tune Complete");
        tel.addData("Ku", ku);
        tel.addData("Tu", tu);
        tel.addData("Kp", kp);
        tel.addData("Ki", ki);
        tel.addData("Kd", kd);
        tel.update();

        stopTelemetryUpdate = true;
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
            tel.addLine("Failed to write PID values to file");
            stopTelemetryUpdate = true;
            tel.update();
        }
    }
}
