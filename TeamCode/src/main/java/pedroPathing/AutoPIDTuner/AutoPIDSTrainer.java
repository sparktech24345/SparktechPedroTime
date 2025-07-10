package pedroPathing.AutoPIDTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name = "AutoPIDSTrainer", group = "Tools")
@com.acmerobotics.dashboard.config.Config
@Disabled
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
    public static final double numberOfCycles = 20;  // Training cycles, default is 20

    public static boolean wasPressedBegging = false;
    public static boolean canAutoTrain = false;
    public static boolean hasNotFullPowered = true;
    public static boolean emergencyReturned = false;
    public static boolean autoTrainingFailed = false;
    public static boolean stopTelemetryUpdate = false;
    public static boolean wasBPressed = false;
    public static boolean hasTrained = false;
    public static double gravityCompensation = 0;


    DcMotor testMotor;
    DcMotor auxMotor;
    double maxDistanceTicks = 0;
    long maxSpeedTimeMs = 0;
    MultipleTelemetry tel;

    String pidFilename = "/sdcard/FIRST/pid_values.txt";

    void motorsSetPower(double power) {
        testMotor.setPower(power);
        //auxMotor.setPower(power);
    }



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
        hasTrained = false;
        gravityCompensation = 0; //got bored of gravity not yed added

        //---------------MOTOR DECLARATION----------------------\\

        testMotor = hardwareMap.get(DcMotor.class, "outakeleftmotor");
        //testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        auxMotor = hardwareMap.dcMotor.get("outakerightmotor");
        auxMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //-------------SERVO STUFF ( OPTIONAL )-----------------\\
        //Servo intakeRotateServo = hardwareMap.get(Servo.class, "intakeRotateServo");
        //intakeRotateServo.setPosition((30) / 228);

        //Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        //outakeArmServo.setPosition(90 / 328);

        //-------------- TELEMETRY ---------------\\
        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        tel.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
        tel.update();

        //actual teleop & stufff

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


            if(gamepad1.b) wasBPressed = true;
            if (hasTrained && !gamepad1.b && wasBPressed) {
                tel.clearAll();
                tel.update();
                runOscillationAutotuneBrutal();
                wasBPressed = false;
            }


            tel.addLine("Ready. Press Gamepad1 A to begin MAX POWER TEST and then press again to begin autotrain");
            tel.addLine("waiting for button press to begin");
            if(emergencyReturned) {
                tel.addLine("EMERGENCY OCCURRED !!!");
                tel.addLine("EMERGENCY OCCURRED !!!");
                tel.addLine("EMERGENCY OCCURRED !!!");
                motorsSetPower(0);
                testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if(autoTrainingFailed) tel.addLine("auto Training Might Have Failed ( VALUES = 0 ) ");

            if(!stopTelemetryUpdate) tel.update();
        }


    }

    void fullPowerRun() {
        //presets
        motorsSetPower(0.7); //not actually full power
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
        motorsSetPower(0);
        maxDistanceTicks = Math.abs(testMotor.getCurrentPosition());
        maxSpeedTimeMs = System.currentTimeMillis() - startTime;

    }


    void returnMotor() {
        tel.addLine("Returning motor to start position at 50% power...");
        tel.update();

        motorsSetPower(-0.5);// Reverse at half power
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
        }

        motorsSetPower(0);
    }

    void runOscillationAutotune() {
        double p = 0.01;
        double step = 0.005;
        double minStep = 0.0001;
        double ku = 0;
        double tu = 0;
        hasTrained = true;

        double target = maxDistanceTicks / 2.0;
        if (target == 0) {
            emergencyReturned = true;
            return;
        }

        boolean pidCalculated = false;
        boolean increasing = true;
        double lastOscillationAmplitude = Double.MAX_VALUE;

        for (int trial = 0; trial < numberOfCycles && opModeIsActive(); trial++) {
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
                motorsSetPower(output);

                oscCycles++;
                if (oscCycles > maxOscillationCycles) break;
                tel.addData("pos",pos);
                tel.addData("target",target);
                tel.addData("error",error);
                tel.addData("power",output);
                tel.addData("P term",p);
                tel.update();
            }

            motorsSetPower(0);

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
            motorsSetPower(-0.5);
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
            }

            motorsSetPower(0);

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
            tel.addLine("This might be  problem beacouse of motors beeing too slow / precise for oscilation");
            tel.addLine("Use Bang Bang Control or the Given P factor to control you motors");
            tel.addLine("You can also try giving it more cycles for another attempt");
            tel.update();
            stopTelemetryUpdate = true; //already finished and its not great
            return;
        }

        double kp = 0.6 * ku;
        double ki = 2 * kp / tu;
        double kd = kp * tu / 8;

        savePIDValues(kp, ki, kd);

        tel.addLine("PID Tune Complete");
        tel.addData("Ku ( ignore ) ", ku);
        tel.addData("Tu ( ignore )", tu);
        tel.addData("Kp", kp);
        tel.addData("Ki NOT RECOMENDED FOR USE", ki);
        tel.addData("Kd", kd);
        tel.update();

        stopTelemetryUpdate = true;
    }




    void savePIDValues(double kp, double ki, double kd) {
        //file creation for value storing
        File pidFile = new File(pidFilename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(pidFile,true))) {
            writer.write("\n\n");
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

    void runAggressivePIDTraining() {
        double target = maxDistanceTicks / 2.0;
        if (target == 0) {
            emergencyReturned = true;
            return;
        }

        motorsSetPower(0);
        sleep(250);

        long startTime = System.currentTimeMillis();
        double maxError = 0;
        boolean reachedTarget = false;

        while (System.currentTimeMillis() - startTime < 5000 && opModeIsActive()) {
            double pos = testMotor.getCurrentPosition();
            double error = target - pos;


            if (pos >= target) {
                reachedTarget = true;
                if (Math.abs(error) > maxError) {
                    maxError = Math.abs(error);
                }
            }

            double power = pos < target ? 1.0 : -1.0;
            motorsSetPower(power);

            if(reachedTarget) break;
        }

        motorsSetPower(0);

        if (!reachedTarget) {
            tel.addLine("Target not reached, defaulting Kp");
            tel.update();
            return;
        }

        // Calculate kp such that kp * maxError = 1.0 (full power)
        double calculatedKp = 1.0 / maxError;

        // Clamp Kp to reasonable range
        calculatedKp = Math.max(0.0001, Math.min(1.0, calculatedKp));

        tel.addLine("Bang-Bang Reverse-Engineered Kp");
        tel.addData("Max Overshoot Error", maxError);
        tel.addData("Calculated Kp", calculatedKp);
        tel.update();

        stopTelemetryUpdate = true;
    }


    void runOscillationAutotuneBrutal() {
        double p = 0.01;
        double step = 0.005;
        double minStep = 0.0001;
        double ku = 0;
        double tu = 0;
        hasTrained = true;

        double target = maxDistanceTicks / 2.0;
        if (target == 0) {
            emergencyReturned = true;
            return;
        }

        boolean pidCalculated = false;
        boolean increasing = true;
        double lastOscillationAmplitude = Double.MAX_VALUE;

        for (int trial = 0; trial < numberOfCycles && opModeIsActive(); trial++) {
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
                motorsSetPower(output);

                oscCycles++;
                if (oscCycles > maxOscillationCycles) break;
                tel.addData("pos",pos);
                tel.addData("target",target);
                tel.addData("error",error);
                tel.addData("power",output);
                tel.addData("P term trial",p);
                tel.update();
            }

            motorsSetPower(0);

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
            motorsSetPower(-0.5);
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
            }

            motorsSetPower(0);

            // Adjust p for next trial
            if (increasing) {
                p += step;
                p *= 1.2;
            } else {
                p += step;
                if (p < 0) p = 0.001;  // avoid negative or zero p
            }
        }

        if (!pidCalculated || ku == 0 || tu == 0) {
            autoTrainingFailed = true;
            tel.addLine("Brutal training also failed -- Use bang bang control");
            tel.update();
            stopTelemetryUpdate = true; //already finished and its not great
            return;
        }

        double kp = 0.6 * ku;
        double ki = 2 * kp / tu;
        double kd = kp * tu / 8;

        savePIDValues(kp, ki, kd);

        tel.addLine("PID Tune Complete");
        tel.addData("Ku ( ignore ) ", ku);
        tel.addData("Tu ( ignore )", tu);
        tel.addData("Kp", kp);
        tel.addData("Ki VERY VERY VERY MUCH NOT RECOMENDED FOR USE", ki);
        tel.addData("Kd", kd);
        tel.update();

        stopTelemetryUpdate = true;
    }






}
