package Experimental.HelperClasses;

import android.util.Pair;

import java.util.HashMap;

public class RobotState {
    private final HashMap<String, String> positions = new HashMap<>();

    public RobotState(Pair<String, String>... pairs) {
        for (Pair<String, String> p : pairs) {
            positions.put(p.first, p.second);
        }
    }

    public HashMap<String, String> getPositions() {
        return positions;
    }
}
