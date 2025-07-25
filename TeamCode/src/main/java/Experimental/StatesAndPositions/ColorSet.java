package Experimental.StatesAndPositions;

import static Experimental.HelperClasses.GlobalStorage.*;

import com.google.blocks.ftcrobotcontroller.runtime.Block;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.R;

public enum ColorSet {
    Red,
    Green,
    Blue,
    Yellow,
    Purple,
    Cyan,
    White,
    Black;

    public static boolean isSample(ColorSet color) {
        switch (color) {
            case Red:
            case Blue:
            case Yellow:
                return true;
        }
        return false;
    }

    public static boolean sampleIsValid(ColorSet color, boolean isYellowValid) {
        if (isYellowValid && color == Yellow)
            return true;
        return currentTeam == color;
    }
    public static ColorSet getColor(NormalizedRGBA colors) {
        return getColor(colors.red, colors.green, colors.blue);
    }
    public static ColorSet getColor(double r, double g, double b) {
        return getColor(r, g, b, 0.7);
    }
    public static ColorSet getColor(double r, double g, double b, double threshold) {
        boolean _r = r >= threshold;
        boolean _g = g >= threshold;
        boolean _b = b >= threshold;
        if (_r && _g && _b) return White;
        if (_r && _g)       return Yellow;
        if (_r && _b)       return Purple;
        if (_g && _b)       return Cyan;
        if (_r)             return Red;
        if (_g)             return Green;
        if (_b)             return Blue;
                            return Black;
    }
}
