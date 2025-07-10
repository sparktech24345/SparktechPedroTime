package pedroPathing.customUtilities.autoRecorder;

public class PoseData {
    public double x;
    public double y;
    public double heading;

    public PoseData(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() { return x; }
    public void setX(double x) { this.x = x; }

    public double getY() { return y; }
    public void setY(double y) { this.y = y; }

    public double getHeading() { return heading; }
    public void setHeading(double heading) { this.heading = heading; }
}
