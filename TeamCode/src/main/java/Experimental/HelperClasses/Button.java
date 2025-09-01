package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.eval;

import android.service.controls.actions.BooleanAction;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Button {
    private final BooleanSupplier condB;
    private final DoubleSupplier condD;
    private boolean WasPressed = false;
    private final ElapsedTime HoldTimer = new ElapsedTime();


    public boolean IsToggledOnPress = false;
    public boolean IsToggledAfterPress = false;
    public boolean IsHeld = false;
    public double HoldTimeMs = 0;
    public boolean ExecuteOnPress = false;
    public boolean ExecuteAfterPress = false;

    public void Toggle() {
        IsToggledOnPress = true;
        IsToggledAfterPress = true;
    }
    public void UnToggle() {
        IsToggledOnPress = false;
        IsToggledAfterPress = false;
    }

    public Button(BooleanSupplier execCond) {
        this.condB = execCond;
        this.condD = null;
    }

    public Button(DoubleSupplier execCond) {
        this.condB = null;
        this.condD = execCond;
    }


    public double raw() {
        try {
            return condD.getAsDouble();
        } catch (NullPointerException e1) {
            try {
                return eval(condB.getAsBoolean());
            } catch (NullPointerException e2) {
                return 0;
            }
        }
    }

    public void update() {
        boolean GamepadInput = (condB == null ? eval(condD.getAsDouble()) : condB.getAsBoolean());
        ExecuteOnPress = false;
        ExecuteAfterPress = false;

        if (!WasPressed && GamepadInput) {
            ExecuteOnPress = true;
            HoldTimer.reset();
            IsToggledOnPress = !IsToggledOnPress;
        }
        if (WasPressed && !GamepadInput) {
            ExecuteAfterPress = true;
            HoldTimeMs = HoldTimer.milliseconds();
            IsToggledAfterPress = !IsToggledAfterPress;
        }
        IsHeld = GamepadInput;
        WasPressed = GamepadInput;
    }
}
