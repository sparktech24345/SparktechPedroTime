package Experimental.HelperClasses;

import android.util.Pair;

import java.util.HashMap;

public class RobotState {
    private final HashMap<String, String> positions = new HashMap<>();

    public RobotState(Pair<String, String>... args) {
        for (Pair<String, String> pair : args) {
            positions.put(pair.first, pair.second);
        }
    }

    public HashMap<String, String> getPositions() {
        return positions;
    }
}
