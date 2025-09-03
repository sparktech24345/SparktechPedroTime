package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.Components.Component;
import Experimental.HelperClasses.Components.MotorComponent;
import Experimental.HelperClasses.Components.ServoComponent;
import Experimental.HelperClasses.ExtendedLinearOpMode;
import Experimental.HelperClasses.GlobalStorage;
import Experimental.HelperClasses.OpMode;
import Experimental.HelperClasses.RobotController;
import Experimental.HelperClasses.RobotState;

import static Experimental.HelperClasses.GlobalStorage.*;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name = "Main TeleOP", group = "Experimental")
public class MainTeleOP extends ExtendedLinearOpMode {

    private RobotController robot = null;
    @Override
    public void runOpMode() {
        setInstances();
        // init
        robot = new RobotController();
        MakeComponents();
        MakeStates();
        robot.init(OpMode.TeleOP);
        robot.UseDefaultMovement();

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

    private void MakeStates() {
        robot.getComponent("INTAKE_EXTENSION")
                .addState("EXTENDED1", 112)
                .addState("EXTENDED2", 245)
                .addState("EXTENDED3", 377)
                .addState("EXTENDED4", 585)
                .addState("RETRACTED", 0)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_EXTENSION")
                .addState("MAX_HIGH_BASKET", 2100)
                .addState("MAX_LOW_BASKET", 800)
                .addState("SPECIMEN_HANG", 1100)
                .addState("AUTO_SPECIMEN_HANG", 950)
                .addState("WALL_PICKUP", 710)
                .addState("STANDBY", 1000)
                .addState("PARKED", 655)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_ARM")
                .addState("WALL_PICKUP", 285)
                .addState("HIGH_RUNG", 170)
                .addState("BASKET_SCORE", 43)
                .addState("TRANSFER_POS", 201)
                .addState("STANDBY_POS", 170)
                .addState("PARKED_POS", 164.84)
                .addState("ABSOLUTE_ZERO", 0);

        robot
                .addState("START_STATE", new RobotState(
                        new Pair<>("INTAKE_EXTENSION", "RETRACTED"),
                        new Pair<>("OUTTAKE_EXTENSION", "STANDBY")
                ))
                .addState("TRANSFER_POS", new RobotState(
                        new Pair<>("OUTTAKE_EXTENSION", "TRANSFER"),
                        new Pair<>("INTAKE_EXTENSION", "RETRACTED")
                ))
                .addState("INTAKE_EXTENSION1", new RobotState(
                        new Pair<>("INTAKE_EXTENSION", "EXTENDED1")
                ))
                .addState("INTAKE_EXTENSION2", new RobotState(
                        new Pair<>("INTAKE_EXTENSION", "EXTENDED2")
                ))
                .addState("INTAKE_EXTENSION3", new RobotState(
                        new Pair<>("INTAKE_EXTENSION", "EXTENDED3")
                ))
                .addState("INTAKE_EXTENSION4", new RobotState(
                        new Pair<>("INTAKE_EXTENSION", "EXTENDED4")
                ));
    }

    private void MakeComponents() {
        robot
            .makeComponent("INTAKE_EXTENSION", new MotorComponent()
                .addMotor(intakeExtendName)
                .useWithPIDController(true)
                .setPIDconstants(0.009, 0.06691449814126393, 0.000302625)
                .setRange(585)
            )
            .makeComponent("OUTTAKE_EXTENSION", new MotorComponent()
                .addMotor(outtakeExtendLeftName)
                .addMotor(outtakeExtendRightName)
                .useWithPIDController(true)
                .setPIDconstants(0.0105, 0.06691449814126393, 0.000112875)
                .setRange(2100)
            )
            .makeComponent("OUTTAKE_ARM", new ServoComponent()
                .addMotor(outtakeArmName)
                .setResolution(328)
            );
        ((MotorComponent)robot.getComponent("INTAKE_EXTENSION")).getMotor(intakeExtendName).setDirection(DcMotorSimple.Direction.REVERSE);
        ((MotorComponent)robot.getComponent("OUTTAKE_EXTENSION")).getMotor(outtakeExtendRightName).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
