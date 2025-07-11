package pedroPathing.AutoPIDTuner;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@Config
public class AutoPIDTuner {
    //---------------MOTOR METHOD DECLARATION----------------------\\

    DcMotor mainMotor,auxMotor1;
    TrainAutoPIDBaseTeleopHandler baseOPMode;
    void setMotors(){
        this.mainMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        this.auxMotor1 = hardwareMap.dcMotor.get("outakerightmotor");
        this.auxMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //motors should have a positive power for extending, thi can be modified by REVERSING the motor like above
    void motorsSetPower(double power) {
        mainMotor.setPower(power);
        auxMotor1.setPower(power); //optional motors if you have more connected to the same system
    }


    //-------------SERVO STUFF ( OPTIONAL )-----------------\\
    void setServos(){
        Servo servo1 = hardwareMap.get(Servo.class, "outakeArmServo");
        servo1.setPosition((double) 150 / 328);
        Servo servo2 = hardwareMap.get(Servo.class, "intakeRotateServo");
        servo2.setPosition((double) (30) / 228);
    }

//-------------FILE SAVING STUFF ( OPTIONAL )-----------------\\

    @SuppressLint("SdCardPath")
    String pidFilename = "/sdcard/FIRST/pid_values.txt";


    HardwareMap hardwareMap;

    MultipleTelemetry tel;
    public AutoPIDTuner(HardwareMap hardwareMap,TrainAutoPIDBaseTeleopHandler baseOPMode, MultipleTelemetry multipleTelemetry){
        this.hardwareMap = hardwareMap;
        this.baseOPMode = baseOPMode;
        this.tel = multipleTelemetry;
        setMotors();
    }

    //-------------CONSTANTS ( MOSTLY )-----------------\\
    
    public double stillThresholdTicks = 2; // Minimum tick change to count as motion
    public long stillTimeoutMs = 20;
    public int maxOscillationCycles = 400;
    public final double numberOfCycles = 20;  // Training cycles, default is 20
    public boolean emergencyReturned = false;
    public boolean autoTrainingFailed = false;
    public boolean hasTrained = false;
    double maxDistanceTicks = 0;
    long maxSpeedTimeMs = 0;

    //------------- File Saving -----------------\\

    void savePIDValues(double kp, double ki, double kd) {
        //file creation for value storing
        File pidFile = new File(pidFilename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(pidFile,true))) {
            writer.write("\n\n");
            @SuppressLint("SimpleDateFormat")
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd hh:mm:ss"); // date and time might crash
            writer.write(sdf.format(new Date()));
            writer.newLine();
            writer.write("Kp=" + kp + "\n");
            writer.write("Ki=" + ki + "\n");
            writer.write("Kd=" + kd + "\n");
        } catch (IOException e) {
            tel.addLine("Failed to write PID values to file");
            tel.update();
        }
    }


    //------------- MOTOR UP AND DOWN METHODS -----------------\\


    void fullPowerRun() {
        //presets
        motorsSetPower(0.7); //not actually full power
        long startTime = System.currentTimeMillis();
        long lastMoveTime = startTime;
        int lastPosition = 0;

        //will run while active and will break if it stands still
        while (baseOPMode.opModeIsActive()) {
            int currentPosition = mainMotor.getCurrentPosition();
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
            tel.addData("Curent Motor Position",mainMotor.getCurrentPosition());
            tel.update();
        }

        //outputs are public static
        motorsSetPower(0);
        maxDistanceTicks = Math.abs(mainMotor.getCurrentPosition());
        maxSpeedTimeMs = System.currentTimeMillis() - startTime;

    }


    void returnMotor() {

        motorsSetPower(-0.5);// Reverse at half power
        long startTime = System.currentTimeMillis();
        long lastMoveTime = startTime;
        int lastPosition = mainMotor.getCurrentPosition();

        while (baseOPMode.opModeIsActive()) {
            int currentPosition = mainMotor.getCurrentPosition();
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





    //------------- TUNING METHODS -----------------\\




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

        for (int trial = 0; trial < numberOfCycles && baseOPMode.opModeIsActive(); trial++) {
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
            while (baseOPMode.opModeIsActive() && (System.currentTimeMillis() - startTime < 10000)) {
                int pos = mainMotor.getCurrentPosition();
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
            int lastPos = mainMotor.getCurrentPosition();

            while (baseOPMode.opModeIsActive() && System.currentTimeMillis() - retractStart < 5000) {
                int currPos = mainMotor.getCurrentPosition();

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
            tel.update();//already finished and its not great
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

        for (int trial = 0; trial < numberOfCycles && baseOPMode.opModeIsActive(); trial++) {
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
            while (baseOPMode.opModeIsActive() && (System.currentTimeMillis() - startTime < 10000)) {
                int pos = mainMotor.getCurrentPosition();
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
            int lastPos = mainMotor.getCurrentPosition();

            while (baseOPMode.opModeIsActive() && System.currentTimeMillis() - retractStart < 5000) {
                int currPos = mainMotor.getCurrentPosition();

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
            tel.update(); //already finished and its not great
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
    }
    
    
    
    
    
}
