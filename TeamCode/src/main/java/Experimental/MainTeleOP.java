package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.ExtendedLinearOpMode;
import Experimental.HelperClasses.GlobalStorage;
import Experimental.HelperClasses.OpMode;
import Experimental.HelperClasses.RobotController;

@TeleOp(name = "Main TeleOP", group = "Experimental")
public class MainTeleOP extends ExtendedLinearOpMode {

    private RobotController robot = null;
    @Override
    public void runOpMode() {
        // init
        setInstances();
        robot = new RobotController();
        robot.init(OpMode.TeleOP);

        while (!isStarted() && !isStopRequested()) {
            //init loop
            robot.init_loop();
        }
        //start

        while (opModeIsActive()) {
            //loop
            robot.loop();
        }
        //stop

    }
}
