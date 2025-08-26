package pedroPathing.AutoPIDTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Auto PID Tuner integrated with Pedro Pathing Follower and goBILDA Pinpoint localizer.
 *
 * - Uses Ziegler–Nichols to find Ku/Tu for Drive (translation) and Heading (rotation)
 * - After computing gains, *directly updates* the Pedro Follower via setDrivePIDF and setHeadingPIDF
 * - Works with PinpointLocalizer (requires the Pinpoint hardware + localizer class available in your project)
 *
 * HOW TO USE
 * 1. Put this OpMode in your teamcode and ensure Pedro Pathing and PinpointLocalizer are available on the classpath.
 * 2. Edit the CONFIG block below for safe power limits & distances.
 * 3. Run OpMode, press A for DRIVE tune and B for HEADING tune. Press X to save the computed gains into
 *    /sdcard/FIRST/pedro_pid.json AND apply them to your Follower instance immediately.
 */
@TeleOp(name = "Pedro_PidTuner_With_Pinpoint", group = "Tools")
@com.acmerobotics.dashboard.config.Config
public class PedroPidTunerWithPinpoint extends LinearOpMode {

    // ----------------- Config -----------------
    public static double DRIVE_TEST_DISTANCE_MM = 500.0;
    public static double HEADING_TEST_DEG = 180.0;
    public static double P_START = 0.01;
    public static double P_STEP = 0.01;
    public static double P_MIN_STEP = 0.0005;
    public static double DRIVE_POWER_CLAMP = 0.55;
    public static double TURN_POWER_CLAMP = 0.45;
    public static int REQUIRED_ZERO_CROSSINGS = 6;
    public static double MAX_TEST_SECONDS = 10.0;
    public static double MAX_TOTAL_SECONDS = 120.0;

    private MultipleTelemetry telemetryOut;
    private ElapsedTime runtime = new ElapsedTime();

    // Pedro & localizer
    private Follower follower;
    private PinpointLocalizer pinpoint;

    // Results
    private Gains driveGains = new Gains();
    private Gains headingGains = new Gains();

    private static final String FILE_PATH = "/sdcard/FIRST/pedro_pid.json";

    private static class Gains {
        double Ku=0, Tu=0, Kp=0, Ki=0, Kd=0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryOut = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryOut.addLine("Initializing Pinpoint localizer and Pedro Follower...");
        telemetryOut.update();

        // Initialize Pinpoint localizer (uses hardwareMap internally). If user already has their own localizer
        // you can supply it by changing this code to accept it.
        try {
            pinpoint = new PinpointLocalizer(hardwareMap);
        } catch (Exception e) {
            telemetryOut.addLine("PinpointLocalizer init FAILED: " + e.getMessage());
            telemetryOut.update();
            return;
        }

        // Create follower using hardware map and the pinpoint localizer
        follower = new Follower(hardwareMap, pinpoint, FollowerConstants.class, FollowerConstants.class);
        follower.initialize(pinpoint);

        telemetryOut.addLine("Ready. A=DriveZN, B=HeadingZN, X=Save+Apply, Y=Emergency Stop");
        telemetryOut.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.y) {
                // emergency
                stopAllMotors();
            }

            if (gamepad1.a) {
                telemetryOut.addLine("Starting DRIVE ZN tune...");
                telemetryOut.update();
                tuneDriveZN();
            }

            if (gamepad1.b) {
                telemetryOut.addLine("Starting HEADING ZN tune...");
                telemetryOut.update();
                tuneHeadingZN();
            }

            if (gamepad1.x) {
                telemetryOut.addLine("Saving and applying gains to Pedro Follower...");
                applyAndSaveGains();
            }

            // Live output
            telemetryOut.addData("Drive Kp,Ki,Kd", format(driveGains.Kp) + ", " + format(driveGains.Ki) + ", " + format(driveGains.Kd));
            telemetryOut.addData("Head  Kp,Ki,Kd", format(headingGains.Kp) + ", " + format(headingGains.Ki) + ", " + format(headingGains.Kd));
            telemetryOut.update();
            idle();
        }
    }

    // ---------------- ZN Drive Tuner (uses average encoder distance as error) ----------------
    private void tuneDriveZN() {
        resetEncoders();
        double targetTicks = mmToTicks(DRIVE_TEST_DISTANCE_MM);
        double p = P_START;
        double step = P_STEP;
        double lastAmp = Double.MAX_VALUE;
        boolean increasing = true;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < MAX_TOTAL_SECONDS) {
            OscMeas m = runDriveOscTrial(p, targetTicks);
            if (!m.valid) break;

            if (m.zeroCrossings >= REQUIRED_ZERO_CROSSINGS && m.sustained) {
                driveGains.Ku = p;
                driveGains.Tu = m.avgPeriodS;
                break;
            }

            if (m.amplitude > lastAmp) {
                increasing = !increasing;
                step = Math.max(step * 0.5, P_MIN_STEP);
            }
            lastAmp = m.amplitude;

            // tiny retract
            setDrivePower(-Math.signum(avgTicks()) * 0.35);
            sleep(250);
            stopAllMotors();

            p = Math.max(0.0005, p + (increasing ? step : -step));
            telemetryOut.addData("trial P", format(p));
            telemetryOut.addData("amplitude", format(m.amplitude));
            telemetryOut.addData("zeroX", m.zeroCrossings);
            telemetryOut.update();
        }

        if (driveGains.Ku <= 0 || driveGains.Tu <= 0) {
            telemetryOut.addLine("Drive ZN failed — adjust P_STEP or distance");
            telemetryOut.update();
            return;
        }

        computeZN(driveGains);
        telemetryOut.addLine("Drive ZN complete");
        telemetryOut.addData("Kp,Ki,Kd", format(driveGains.Kp) + ", " + format(driveGains.Ki) + ", " + format(driveGains.Kd));
        telemetryOut.update();
    }

    // ---------------- ZN Heading Tuner (uses Pinpoint/IMU heading) ----------------
    private void tuneHeadingZN() {
        double p = P_START;
        double step = P_STEP;
        double lastAmp = Double.MAX_VALUE;
        boolean increasing = true;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < MAX_TOTAL_SECONDS) {
            OscMeas m = runHeadingOscTrial(p, HEADING_TEST_DEG);
            if (!m.valid) { telemetryOut.addLine("Heading trial invalid (no IMU/localizer)"); telemetryOut.update(); return; }

            if (m.zeroCrossings >= REQUIRED_ZERO_CROSSINGS && m.sustained) {
                headingGains.Ku = p;
                headingGains.Tu = m.avgPeriodS;
                break;
            }

            if (m.amplitude > lastAmp) {
                increasing = !increasing;
                step = Math.max(step * 0.5, P_MIN_STEP);
            }
            lastAmp = m.amplitude;

            stopAllMotors();
            p = Math.max(0.0005, p + (increasing ? step : -step));

            telemetryOut.addData("trial P", format(p));
            telemetryOut.addData("amplitude", format(m.amplitude));
            telemetryOut.addData("zeroX", m.zeroCrossings);
            telemetryOut.update();
        }

        if (headingGains.Ku <= 0 || headingGains.Tu <= 0) {
            telemetryOut.addLine("Heading ZN failed — try higher P range or check Pinpoint/IMU");
            telemetryOut.update();
            return;
        }

        computeZN(headingGains);
        telemetryOut.addLine("Heading ZN complete");
        telemetryOut.addData("Kp,Ki,Kd", format(headingGains.Kp) + ", " + format(headingGains.Ki) + ", " + format(headingGains.Kd));
        telemetryOut.update();
    }

    // ---------------- Oscillation trials ----------------
    private static class OscMeas { boolean valid=false; double amplitude=0; int zeroCrossings=0; int zeroxPeriods=0; double avgPeriodS=0; boolean sustained=false; }

    private OscMeas runDriveOscTrial(double p, double targetTicks) {
        OscMeas m = new OscMeas(); m.valid=true;
        resetEncoders();
        double lastErr = targetTicks - avgTicks();
        long lastZero = 0; int periods=0; double sum=0;
        double maxAbs=0, minAbs=Double.MAX_VALUE; int zeroX=0;

        ElapsedTime trial = new ElapsedTime();
        while (opModeIsActive() && trial.seconds() < MAX_TEST_SECONDS) {
            double err = targetTicks - avgTicks();
            double out = clip(p * err, -DRIVE_POWER_CLAMP, DRIVE_POWER_CLAMP);
            setDrivePower(out, out);

            double absErr = Math.abs(err);
            maxAbs = Math.max(maxAbs, absErr);
            minAbs = Math.min(minAbs, absErr);

            if (Math.signum(err) != Math.signum(lastErr)) {
                zeroX++;
                long now = System.nanoTime();
                if (lastZero != 0) { sum += (now - lastZero) / 1e9; periods++; }
                lastZero = now;
            }
            lastErr = err;

            if (zeroX >= REQUIRED_ZERO_CROSSINGS) break;
            idle();
        }

        stopAllMotors();
        m.zeroCrossings = zeroX; m.zeroxPeriods = periods; m.avgPeriodS = periods>0?sum/periods:0; m.amplitude = maxAbs - minAbs;
        m.sustained = m.amplitude > 0.02 * Math.abs(targetTicks);
        return m;
    }

    private OscMeas runHeadingOscTrial(double p, double targetDeg) {
        OscMeas m = new OscMeas();
        m.valid = (pinpoint != null);
        if (!m.valid) return m;

        double lastErr = wrapDeg(targetDeg - pinpoint.getPose().getHeading());
        long lastZero = 0; int periods=0; double sum=0;
        double maxAbs=0, minAbs=Double.MAX_VALUE; int zeroX=0;

        ElapsedTime trial = new ElapsedTime();
        while (opModeIsActive() && trial.seconds() < MAX_TEST_SECONDS) {
            double err = wrapDeg(targetDeg - pinpoint.getPose().getHeading());
            double out = clip(p * err, -TURN_POWER_CLAMP, TURN_POWER_CLAMP);
            setDrivePower(out, -out);

            double absErr = Math.abs(err);
            maxAbs = Math.max(maxAbs, absErr);
            minAbs = Math.min(minAbs, absErr);

            if (Math.signum(err) != Math.signum(lastErr)) {
                zeroX++;
                long now = System.nanoTime();
                if (lastZero != 0) { sum += (now - lastZero) / 1e9; periods++; }
                lastZero = now;
            }
            lastErr = err;

            if (zeroX >= REQUIRED_ZERO_CROSSINGS) break;
            idle();
        }

        stopAllMotors();
        m.zeroCrossings = zeroX; m.zeroxPeriods = periods; m.avgPeriodS = periods>0?sum/periods:0; m.amplitude = maxAbs - minAbs;
        m.sustained = m.amplitude > 0.02 * Math.abs(targetDeg);
        return m;
    }

    // ---------------- Compute ZN gains ----------------
    private void computeZN(Gains g) {
        g.Kp = 0.6 * g.Ku;
        g.Ki = 2.0 * g.Kp / g.Tu;
        g.Kd = g.Kp * g.Tu / 8.0;
    }

    // ---------------- Apply to Pedro Follower & save ----------------
    private void applyAndSaveGains() {
        // Apply to follower
        try {
            CustomFilteredPIDFCoefficients driveCoeffs = new CustomFilteredPIDFCoefficients(driveGains.Kp, driveGains.Ki, driveGains.Kd, 0.6, 0.0);
            CustomPIDFCoefficients headingCoeffs = new CustomPIDFCoefficients(headingGains.Kp, headingGains.Ki, headingGains.Kd, 0.0);
            headingCoeffs.setCoefficients(headingGains.Kp, headingGains.Ki, headingGains.Kd, 0.0);

            follower.setDrivePIDF(driveCoeffs);
            follower.setHeadingPIDF(headingCoeffs);
        } catch (Exception e) {
            telemetryOut.addLine("Failed to apply gains to follower: " + e.getMessage());
            telemetryOut.update();
        }

        // Save JSON
        String json = "{ " + "  \"drive\": { \"Kp\": " + f(driveGains.Kp) +
                ", \"Ki\": " + f(driveGains.Ki) + ", \"Kd\": " + f(driveGains.Kd) + " }, " +
                "  \"heading\": { \"Kp\": " + f(headingGains.Kp) + ", \"Ki\": " +
                f(headingGains.Ki) + ", \"Kd\": " + f(headingGains.Kd) + " } " + "} ";

        try (BufferedWriter w = new BufferedWriter(new FileWriter(new File(FILE_PATH)))) {
            w.write(json);
            telemetryOut.addLine("Saved PID to " + FILE_PATH);
        } catch (IOException e) {
            telemetryOut.addLine("Failed to write PID file: " + e.getMessage());
        }
        telemetryOut.update();
    }

    // ---------------- Utility helpers ----------------
    private void setDrivePower(double left, double right) {
        // delegate to follower teleop vectors so Pedro handles motor mapping
        follower.setTeleOpMovementVectors(left, 0, right, false);
    }
    private void setDrivePower(double power) { setDrivePower(power, power); }
    private void stopAllMotors() { setDrivePower(0,0); }

    private void resetEncoders() {
        // Pedro's follower uses its own PoseUpdater that is backed by the localizer. We reset the localizer by re-init.
        try { pinpoint.update(); } catch (Exception ignored) {}
    }

    private double avgTicks() {
        // If your localizer exposes encoder ticks, you can use them. For safety, we use position x (mm) as proxy.
        return mmToTicks(pinpoint.getPose().getX());
    }

    private double mmToTicks(double mm) {
        // Placeholder: Pedro's PinpointLocalizer will handle units; this is only for amplitude heuristics.
        // If you prefer exact tick conversions, plug your robot constants here.
        return mm;
    }

    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrapDeg(double deg) { while (deg>180) deg-=360; while (deg<=-180) deg+=360; return deg; }
    private static String format(double d) { return String.format("%.5f", d); }
    private static String f(double d) { return String.format("%.6f", d); }
}
