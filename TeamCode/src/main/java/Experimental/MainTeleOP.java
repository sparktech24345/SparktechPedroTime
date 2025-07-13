package Experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Experimental.HelperClasses.MainConfig;

@TeleOp(name = "Main TeleOP", group = "Experimental")
public class MainTeleOP extends LinearOpMode {

    private MainConfig cfg = new MainConfig();

    @Override
    public void runOpMode() {
        // init
        cfg.InitAllStates();

        waitForStart();
        //start


        while (opModeIsActive()) {
        //loop

        }
        //stop

    }
}
