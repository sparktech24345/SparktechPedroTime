package Experimental.HelperClasses;

import android.os.Bundle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ComplexGamepad {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public ComplexGamepad() {}
    public ComplexGamepad(Gamepad gpad1, Gamepad gpad2) {
        gamepad1 = gpad1;
        gamepad2 = gpad2;
    }

    public void SetGamepads(Gamepad gpad1, Gamepad gpad2) {
        gamepad1 = gpad1;
        gamepad2 = gpad2;
    }

    // MUST BE INCLUDED IN MAIN LOOP FOR THE Execute, IsHeld and IsToggled BOOLEANS TO WORK
    public void CheckGamepads() {
        A1.RunCheck(gamepad1.a);
        A2.RunCheck(gamepad2.a);

        B1.RunCheck(gamepad1.b);
        B2.RunCheck(gamepad2.b);

        X1.RunCheck(gamepad1.x);
        X2.RunCheck(gamepad2.x);

        Y1.RunCheck(gamepad1.y);
        Y2.RunCheck(gamepad2.y);

        DUP1.RunCheck(gamepad1.dpad_up);
        DUP2.RunCheck(gamepad2.dpad_up);

        DDOWN1.RunCheck(gamepad1.dpad_down);
        DDOWN2.RunCheck(gamepad2.dpad_down);

        DLEFT1.RunCheck(gamepad1.dpad_left);
        DLEFT2.RunCheck(gamepad2.dpad_left);

        DRIGHT1.RunCheck(gamepad1.dpad_right);
        DRIGHT2.RunCheck(gamepad2.dpad_right);

        RIGHT_BUMPER1.RunCheck(gamepad1.right_bumper);
        RIGHT_BUMPER2.RunCheck(gamepad2.right_bumper);

        LEFT_BUMPER1.RunCheck(gamepad1.left_bumper);
        LEFT_BUMPER2.RunCheck(gamepad2.left_bumper);

        RIGHT_TRIGGER1.RunCheck(gamepad1.right_trigger);
        RIGHT_TRIGGER2.RunCheck(gamepad2.right_trigger);

        LEFT_TRIGGER1.RunCheck(gamepad1.left_trigger);
        LEFT_TRIGGER2.RunCheck(gamepad2.left_trigger);

        START1.RunCheck(gamepad1.start);
        START2.RunCheck(gamepad2.start);
    }

    public Button A1 = new Button();
    public Button A2 = new Button();

    public Button B1 = new Button();
    public Button B2 = new Button();

    public Button X1 = new Button();
    public Button X2 = new Button();

    public Button Y1 = new Button();
    public Button Y2 = new Button();

    public Button DUP1 = new Button();
    public Button DUP2 = new Button();

    public Button DDOWN1 = new Button();
    public Button DDOWN2 = new Button();

    public Button DLEFT1 = new Button();
    public Button DLEFT2 = new Button();

    public Button DRIGHT1 = new Button();
    public Button DRIGHT2 = new Button();

    public Button RIGHT_BUMPER1 = new Button();
    public Button RIGHT_BUMPER2 = new Button();

    public Button LEFT_BUMPER1 = new Button();
    public Button LEFT_BUMPER2 = new Button();

    public Button RIGHT_TRIGGER1 = new Button();
    public Button RIGHT_TRIGGER2 = new Button();

    public Button LEFT_TRIGGER1 = new Button();
    public Button LEFT_TRIGGER2 = new Button();

    public Button START1 = new Button();
    public Button START2 = new Button();
}
