package pedroPathing.AutoPIDS;

public class NewPidsController {
    // Persistent variables for PID calculation
    public static double integral = 0;
    public static double lastError = 0;
    public static long lastTime = System.currentTimeMillis();
    public static double kp = 0.009;
    public static double ki = 0.06691449814126393;
    public static double kd = 0.000302625;


    public static double pidController(double target, double current) {
        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTime) / 1000.0;  // in seconds
        lastTime = now;

        double error = target - current;

        // Integral term (accumulated error)
        integral += error * deltaTime;

        // Derivative term (rate of change of error)
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        lastError = error;

        // PID output
        double output = (kp * error) /*+ (ki * integral)*/ + (kd * derivative);

        // Clamp to motor power limits
        output = Math.max(-1.0, Math.min(1.0, output));

        return output;
    }

}
