package Experimental.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Experimental.HelperClasses.RobotController;
import Experimental.HelperClasses.OpModes;


@Autonomous(name = "Main Auto", group = "Experimental")
public class MainAuto extends OpMode {
    private RobotController robot;

    @Override
    public void init() {
//        robot = new RobotController();
        MakeAutoPoses();
        robot.init(OpModes.Autonomous);
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

    private void MakeAutoPoses() {
        robot
                .addAutoPosition("TEST1", 10, 10, 0)
                .addAutoPosition("TEST2", -10, 10, 90)
                .addAutoPosition("TEST3", 10, -10, 180)
                .addAutoPosition("TEST4", 10, 10, 270);
    }
}
