package Experimental.HelperClasses.Components;

import static Experimental.HelperClasses.GlobalStorage.hardwareMapInstance;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public abstract class Component {

    protected HashMap<String, Double> states = new HashMap<String, Double>();
    protected double target = 0;
    protected double resolution = 1;
    protected double range = -1;
    protected boolean moveOnInit = false;

    public <T extends Component> T addState(String s, double v) {
        states.put(s, v);
        return (T) this;
    }

    public <T extends Component> T addState(String s, double v, boolean setAsDefault) {
        states.put(s, v);
        target = v;
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

    public abstract double getPos();

    public <T extends Component> T setResolution(double res) {
        resolution = res;
        return (T) this;
    }

    public boolean moveDuringInit() {
        return moveOnInit;
    }

    public <T extends Component> T moveDuringInit(boolean move) {
        moveOnInit = move;
        return (T) this;
    }

    public abstract void update();
}
