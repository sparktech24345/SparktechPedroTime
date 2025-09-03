package Experimental.HelperClasses.Components;

import static Experimental.HelperClasses.GlobalStorage.hardwareMapInstance;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public abstract class Component {

    protected HashMap<String, Double> states = new HashMap<String, Double>();
    protected double target = 0;
    protected double resolution = 1;
    protected double range = -1;

    public <T extends Component> T addState(String s, double v) {
        states.put(s, v);
        return (T) this;
    }

    public <T extends Component> T setRange(double r) {
        this.range = r;
        return (T) this;
    }

    public <T extends Component> T loadState(String s) {
        target = states.get(s);
        return (T) this;
    }

    public <T extends Component> T setResolution(double res) {
        resolution = res;
        return (T) this;
    }

    public abstract void update();
}
