package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;

public class ComplexGamepad {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private HashMap<String, Button> controlMap = null;

    private ComplexGamepad() {}
    public ComplexGamepad(Gamepad gpad1, Gamepad gpad2) {
        SetGamepads(gpad1, gpad2);
    }

    public void SetGamepads(Gamepad gpad1, Gamepad gpad2) {
        gamepad1 = gpad1;
        gamepad2 = gpad2;
        controlMap = new HashMap<String, Button>(36) {{
            put("A1", new Button(() -> gamepad1.a));
            put("A2", new Button(() -> gamepad2.a));

            put("B1", new Button(() -> gamepad1.b));
            put("B2", new Button(() -> gamepad2.b));

            put("X1", new Button(() -> gamepad1.x));
            put("X2", new Button(() -> gamepad2.x));

            put("Y1", new Button(() -> gamepad1.y));
            put("Y2", new Button(() -> gamepad2.y));

            put("DPAD_UP1", new Button(() -> gamepad1.dpad_up));
            put("DPAD_UP2", new Button(() -> gamepad2.dpad_up));

            put("DPAD_DOWN1", new Button(() -> gamepad1.dpad_down));
            put("DPAD_DOWN2", new Button(() -> gamepad2.dpad_down));

            put("DPAD_RIGHT1", new Button(() -> gamepad1.dpad_right));
            put("DPAD_RIGHT2", new Button(() -> gamepad2.dpad_right));

            put("DPAD_LEFT1", new Button(() -> gamepad1.dpad_left));
            put("DPAD_LEFT2", new Button(() -> gamepad2.dpad_left));

            put("START1", new Button(() -> gamepad1.start));
            put("START2", new Button(() -> gamepad2.start));

            put("BACK1", new Button(() -> gamepad1.back));
            put("BACK2", new Button(() -> gamepad2.back));

            put("LEFT_STICK_X1", new Button(() -> gamepad1.left_stick_x));
            put("LEFT_STICK_X2", new Button(() -> gamepad2.left_stick_x));

            put("LEFT_STICK_Y1", new Button(() -> gamepad1.left_stick_y));
            put("LEFT_STICK_Y2", new Button(() -> gamepad2.left_stick_y));

            put("RIGHT_STICK_X1", new Button(() -> gamepad1.right_stick_x));
            put("RIGHT_STICK_X2", new Button(() -> gamepad2.right_stick_x));

            put("RIGHT_STICK_Y1", new Button(() -> gamepad1.right_stick_y));
            put("RIGHT_STICK_Y2", new Button(() -> gamepad2.right_stick_y));

            put("LEFT_TRIGGER1", new Button(() -> gamepad1.left_trigger));
            put("LEFT_TRIGGER2", new Button(() -> gamepad2.left_trigger));

            put("RIGHT_TRIGGER1", new Button(() -> gamepad1.right_trigger));
            put("RIGHT_TRIGGER2", new Button(() -> gamepad2.right_trigger));

            put("LEFT_BUMPER1", new Button(() -> gamepad1.left_bumper));
            put("LEFT_BUMPER2", new Button(() -> gamepad2.left_bumper));

            put("RIGHT_BUMPER1", new Button(() -> gamepad1.right_bumper));
            put("RIGHT_BUMPER2", new Button(() -> gamepad2.right_bumper));
        }};
    }

    public Button get(String str) {
        return controlMap.get(str);
    }

    // MUST BE INCLUDED IN MAIN LOOP FOR THE BOOLEANS TO WORK
    public void update() {
        for (Button button : controlMap.values()) {
            button.update();
        }
    }
}
