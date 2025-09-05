package Experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Experimental.HelperClasses.Actions.DelayAction;
import Experimental.HelperClasses.Actions.StateAction;
import Experimental.HelperClasses.ComplexGamepad;
import Experimental.HelperClasses.Components.MotorComponent;
import Experimental.HelperClasses.Components.ServoComponent;
import Experimental.HelperClasses.OpModes;
import Experimental.HelperClasses.RobotController;
import Experimental.HelperClasses.RobotState;
import Experimental.StatesAndPositions.ColorSet;

import static Experimental.HelperClasses.GlobalStorage.*;

@TeleOp(name = "Main TeleOP", group = "Experimental")
public class MainTeleOP extends LinearOpMode {

    private RobotController robot = null;
    @Override
    public void runOpMode() {
        // init
        initAll();
        robot = new RobotController();
        MakeComponents();
        MakeStates();
        robot.init(OpModes.TeleOP);
        robot.UseDefaultMovement();
        robot.setControls(controls);

        while (!isStarted() && !isStopRequested()) {
            //init loop
            robot.init_loop();
            if (robot.getControllerKey("LEFT_BUMPER1").IsHeld && robot.getControllerKey("START1").IsHeld)
                currentTeam = ColorSet.Blue;
            else if (robot.getControllerKey("RIGHT_BUMPER1").IsHeld && robot.getControllerKey("START1").IsHeld)
                currentTeam = ColorSet.Red;
        }
        //start

        while (opModeIsActive()) {
            //loop
            robot.loop();
        }
        //stop

    }

    private void MakeComponents() {
        robot
            .makeComponent("INTAKE_EXTENSION", new MotorComponent()
                    .addMotor(intakeExtendName)
                    .useWithPIDController(true)
                    .setPIDconstants(0.009, 0.06691449814126393, 0.000302625)
                    .setRange(585)
                    .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                    .setDirection(intakeExtendName, DcMotorSimple.Direction.REVERSE)
            )
            .makeComponent("INTAKE_PIVOT", new ServoComponent()
                    .addMotor(intakePosName)
                    .setResolution(228)
            )
            .makeComponent("INTAKE_SPIN", new MotorComponent()
                    .addMotor(intakeSpinName)
                    .useWithPIDController(false)
                    .setRange(1)
            )
            .makeComponent("OUTTAKE_EXTENSION", new MotorComponent()
                    .addMotor(outtakeExtendLeftName)
                    .addMotor(outtakeExtendRightName)
                    .useWithPIDController(true)
                    .setPIDconstants(0.0105, 0.06691449814126393, 0.000112875)
                    .setRange(2100)
                    .setBehaviour(DcMotor.ZeroPowerBehavior.BRAKE)
                    .setDirection(outtakeExtendRightName, DcMotorSimple.Direction.REVERSE)
            )
            .makeComponent("OUTTAKE_ARM", new ServoComponent()
                    .addMotor(outtakeArmName)
                    .setResolution(328)
                    .moveDuringInit(true)
            )
            .makeComponent("OUTTAKE_CLAW", new ServoComponent()
                    .addMotor(outtakeClawName)
                    .setResolution(360)
                    .moveDuringInit(true)
            );
    }

    private void MakeStates() {
        robot.getComponent("INTAKE_EXTENSION")
                .addState("EXTENDED1", 112)
                .addState("EXTENDED2", 245)
                .addState("EXTENDED3", 377)
                .addState("EXTENDED4", 585)
                .addState("RETRACTED", 0)
                .addState("ABSOLUTE_ZERO", 0, true);

        robot.getComponent("INTAKE_PIVOT")
                .addState("SAMPLE_PICKUP", 211)
                .addState("SPIT_OUT", 15, true)
                .addState("TRANSFER_POS", 153)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("INTAKE_SPIN")
                .addState("COLLECT", 1)
                .addState("SPIT_OUT", -0.5)
                .addState("OFF", 0, true)
                .addState("SPIT_OUT_FAST", -1)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_EXTENSION")
                .addState("MAX_HIGH_BASKET", 2100)
                .addState("MAX_LOW_BASKET", 800)
                .addState("SPECIMEN_HANG", 1100)
                .addState("AUTO_SPECIMEN_HANG", 950)
                .addState("WALL_PICKUP", 710)
                .addState("STANDBY", 1000)
                .addState("TRANSFER_POS", 0, true)
                .addState("PARKED", 655)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_ARM")
                .addState("WALL_PICKUP", 285)
                .addState("HIGH_RUNG", 170)
                .addState("BASKET_SCORE", 43)
                .addState("TRANSFER_POS", 201)
                .addState("STANDBY_POS", 170, true)
                .addState("PARKED_POS", 164.84)
                .addState("ABSOLUTE_ZERO", 0);

        robot.getComponent("OUTTAKE_CLAW")
                .addState("EXTRA_OPEN", 208)
                .addState("OPEN", 128)
                .addState("CLOSED", 35, true)
                .addState("ABSOLUTE_ZERO", 0);

        robot
                .addRobotState("TRANSFER_POS", new RobotState(
                        make_pair("INTAKE_EXTENSION", "RETRACTED"),
                        make_pair("INTAKE_PIVOT", "TRANSFER_POS"),
                        make_pair("OUTTAKE_EXTENSION", "TRANSFER_POS"),
                        make_pair("OUTTAKE_ARM", "TRANSFER_POS")
                ))
                .addRobotState("SPECIMEN_HANG", new RobotState(
                        make_pair("OUTTAKE_EXTENSION", "SPECIMEN_HANG"),
                        make_pair("OUTTAKE_ARM", "HIGH_RUNG")
                ))
                .addRobotState("STANDBY", new RobotState(
                        make_pair("INTAKE_EXTENSION", "RETRACTED"),
                        make_pair("OUTTAKE_EXTENSION", "STANDBY"),
                        make_pair("OUTTAKE_ARM", "STANDBY_POS")
                ));
    }
    
    private Runnable controls = () -> {
        if (robot.getControllerKey("X1").ExecuteOnPress && !robot.getControllerKey("X1").IsToggledAfterPress) {
            robot.addToQueue(new StateAction(true, "OUTTAKE_EXTENSION", "MAX_HIGH_BASKET"));
            robot.addToQueue(new StateAction(true, "OUTTAKE_ARM", "BASKET_SCORE"));
        }
        else if (robot.getControllerKey("X1").ExecuteOnPress && robot.getControllerKey("X1").IsToggledAfterPress) {
            robot.addToQueue(new StateAction(true, "OUTTAKE_CLAW", "EXTRA_OPEN"));
        }
        else if (robot.getControllerKey("X1").ExecuteAfterPress && !robot.getControllerKey("X1").IsToggledOnPress) {
            robot.addToQueue(new DelayAction(true, 250));
            robot.addToQueue(new StateAction(true, "OUTTAKE_ARM", "STANDBY_POS"));
            robot.addToQueue(new StateAction(true, "OUTTAKE_EXTENSION", "STANDBY"));
        }
        if (robot.getControllerKey("DPAD_UP1").ExecuteOnPress) {
            robot.addToQueue(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED1"));
        }
        else if (robot.getControllerKey("DPAD_RIGHT1").ExecuteOnPress) {
            robot.addToQueue(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED2"));
        }
        else if (robot.getControllerKey("DPAD_DOWN1").ExecuteOnPress) {
            robot.addToQueue(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED3"));
        }
        else if (robot.getControllerKey("DPAD_LEFT1").ExecuteOnPress) {
            robot.addToQueue(new StateAction(true, "INTAKE_EXTENSION", "EXTENDED456"));
        }
        if (robot.getControllerKey("A1").ExecuteOnPress && !robot.getControllerKey("A1").IsToggledAfterPress) {
            robot.addToQueue(new StateAction(true, "INTAKE_PIVOT", "SAMPLE_PICKUP"));
        }
        if (robot.getControllerKey("A1").IsToggledOnPress) {
            if (robot.getControllerKey("RIGHT_BUMPER1").IsHeld) {
                robot.addToQueue(new StateAction(true, "INTAKE_SPIN", "SPIT_OUT"));
            }
            else if (ColorSet.validateSample(ColorSet.Undefined, true)) {
                robot.getControllerKey("A1").UnToggle();
                robot.addToQueue(
                        new StateAction(true, "TRANSFER_POS"), 
                        new StateAction(true, "OUTTAKE_CLAW", "OPEN"), 
                        new DelayAction(true, 200),
                        new StateAction(true, "INTAKE_SPIN", "OFF"), 
                        new DelayAction(true, 200),
                        new StateAction(true, "OUTTAKE_CLAW", "CLOSED"), 
                        new DelayAction(true, 150),
                        new StateAction(true, "SPECIMEN_HANG") 
                );
            } else {
                robot.addToQueue(new StateAction(true, "INTAKE_SPIN", "COLLECT"));
            }
        }
        else robot.addToQueue(new StateAction(true, "INTAKE_SPIN", "OFF")); 
    };

    private Runnable tele = () -> {
        telemetryInstance.addData("INTAKE_EXTENSION", robot.getComponent("INTAKE_EXTENSION").getPos());
        telemetryInstance.addData("INTAKE_PIVOT", robot.getComponent("INTAKE_PIVOT").getPos());
        telemetryInstance.addData("INTAKE_SPIN", robot.getComponent("INTAKE_SPIN").getPos());
        telemetryInstance.addData("OUTTAKE_EXTENSION", robot.getComponent("OUTTAKE_EXTENSION").getPos());
        telemetryInstance.addData("OUTTAKE_ARM", robot.getComponent("OUTTAKE_ARM").getPos());
        telemetryInstance.addData("OUTTAKE_CLAW", robot.getComponent("OUTTAKE_CLAW").getPos());
    };

    private void initAll() {
        ComplexGamepad gamepad = new ComplexGamepad(gamepad1, gamepad2);
        MultipleTelemetry tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadInstance = gamepad;
        hardwareMapInstance = hardwareMap;
        telemetryInstance = tel;
    }
}
