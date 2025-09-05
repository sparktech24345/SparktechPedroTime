package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum ColorSet {
    Undefined,
    Red,
    Blue,
    Yellow;

    public static boolean validateSample(ColorSet current, boolean yellowValid) {
        if (yellowValid && current == Yellow) return true;
        return current == currentTeam;
    }

    public static ColorSet getColor(NormalizedRGBA colors) {
        return getColor(colors.red, colors.green, colors.blue);
    }
    public static ColorSet getColor(double r, double g, double b) {
        if (r < 0.006 && b < 0.004) return Undefined;
        if (r > g && r > b) return Red;
        if (b > g && b > r) return Blue;
        if (g > r && g > b) return Yellow;
        return Undefined;
    }
}
