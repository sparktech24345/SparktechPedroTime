package Experimental.HelperClasses;

import android.util.Pair;

import java.util.HashMap;

public class RobotState {
    private final HashMap<String, String> positions = new HashMap<>();

    public RobotState(String... args) {
        String key = null;
        String value = null;
        int i = 0;
        for (String str : args) {
            if (i % 2 == 0) {
                key = str;
            }
            else {
                value = str;
                positions.put(key, value);
            }
            ++i;
        }
    }

    public HashMap<String, String> getPositions() {
        return positions;
    }
}
