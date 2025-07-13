package Experimental.HelperClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Button {
    private enum RunMode {
        RunOnPress,
        RunAfterPress
    };
    private boolean WasPressed = false;
    private boolean RunOnPress;
    private RunMode CurrentRunMode = RunMode.RunOnPress;
    public boolean IsToggled = false;
    public boolean IsHeld = false;
    public ElapsedTime HoldTimer = new ElapsedTime();
    public boolean Execute = false;

    public void SetRunMode(RunMode runmode) {
        CurrentRunMode = runmode;
    }

    public void RunCheck(boolean GamepadInput) {
        if (CurrentRunMode == RunMode.RunOnPress) {
            Execute = false;
            if (GamepadInput) {
                if (WasPressed) {
                    if (!IsHeld) HoldTimer.reset();
                    IsHeld = true;
                }
                else {
                    WasPressed = true;
                    Execute = true;
                    IsToggled = !IsToggled;
                }
            }
            else { // if !GamepadInput
                WasPressed = false;
                IsHeld = false;
            }
        }
        else {
            Execute = false;
            if (!GamepadInput) {
                if (WasPressed) {
                    Execute = true;
                    IsToggled = !IsToggled;
                    IsHeld = false;
                    WasPressed = false;
                }
            }
            else { // if GamepadImput
                WasPressed = true;
                if (!IsHeld) HoldTimer.reset();
                IsHeld = true;
            }
        }
    }

    public void RunCheck(double GamepadInput) {
        if (CurrentRunMode == RunMode.RunOnPress) {
            Execute = false;
            if (GamepadInput != 0) {
                if (WasPressed) {
                    if (!IsHeld) HoldTimer.reset();
                    IsHeld = true;
                }
                else {
                    WasPressed = true;
                    Execute = true;
                    IsToggled = !IsToggled;
                }
            }
            else { // if !GamepadInput
                WasPressed = false;
                IsHeld = false;
            }
        }
        else {
            Execute = false;
            if (GamepadInput == 0) {
                if (WasPressed) {
                    Execute = true;
                    IsToggled = !IsToggled;
                    IsHeld = false;
                    WasPressed = false;
                }
            }
            else { // if GamepadImput
                WasPressed = true;
                if (!IsHeld) HoldTimer.reset();
                IsHeld = true;
            }
        }
    }
}
