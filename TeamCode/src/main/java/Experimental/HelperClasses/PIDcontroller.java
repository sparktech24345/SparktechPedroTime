package Experimental.HelperClasses;

public class PIDcontroller {
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.currentTimeMillis();

    public PIDcontroller(double p, double i, double d) {
        setConstants(p, i, d);
    }

    public PIDcontroller() {}

    public void setConstants(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }

    public double calculate(double target, double current) {
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
