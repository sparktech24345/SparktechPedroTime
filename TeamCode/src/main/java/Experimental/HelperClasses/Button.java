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
    public double HoldTimeMs = 0;
    private ElapsedTime HoldTimer = new ElapsedTime();
    public boolean Execute = false;

    public void SetRunMode(RunMode runmode) {
        CurrentRunMode = runmode;
    }

    public void RunCheck(boolean GamepadInput) {
        boolean ExecuteOnPress = false;
        boolean ExecuteAfterPress = false;
        Execute = false;

        if (!WasPressed && GamepadInput) {
            ExecuteOnPress = true;
            HoldTimer.reset();
            if (CurrentRunMode == RunMode.RunOnPress)
                IsToggled = !IsToggled;
        }
        if (WasPressed && !GamepadInput) {
            ExecuteAfterPress = true;
            HoldTimeMs = HoldTimer.milliseconds();
            if (CurrentRunMode == RunMode.RunAfterPress)
                IsToggled = !IsToggled;
        }
        Execute = (CurrentRunMode == RunMode.RunOnPress ? ExecuteOnPress : ExecuteAfterPress);
        IsHeld = GamepadInput;
        WasPressed = GamepadInput;
    }

    public void RunCheck(double GamepadInput) {
        RunCheck(GamepadInput > 0);
    }
}
