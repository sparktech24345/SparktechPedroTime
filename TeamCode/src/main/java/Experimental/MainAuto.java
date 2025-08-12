package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.ExtendedOpMode;
import Experimental.HelperClasses.GlobalStorage;
import Experimental.HelperClasses.OpMode;
import Experimental.HelperClasses.RobotController;


@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends ExtendedOpMode {
    private RobotController robot;

    @Override
    public void init() {
        setInstances();
        robot = new RobotController();
        robot.init(OpMode.Autonomous);
    }

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        robot.loop();
    }

    @Override
    public void stop() {
    }
}
