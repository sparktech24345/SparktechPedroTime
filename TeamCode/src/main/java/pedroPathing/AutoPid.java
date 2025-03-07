package pedroPathing;

import static pedroPathing.PositionStorage.intakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPos;
import static pedroPathing.PositionStorage.outakeTargetPosAdder;
import static pedroPathing.PositionStorage.resetStuff;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Motor Testing", group = "Linear OpMode")
public class AutoPid extends LinearOpMode {


    public static double autoKp=0;
    public static double autoKd=0;

    @Override
    public void runOpMode() throws InterruptedException {



        //==============================\\


        int targetPosition = 500; //limit but ith overshoot space
        String motorName = "intakeMotor";
        boolean reverse = true;


        //===============================\\


//
        double overShoot=0;
        double lastOverShoot = 0;
        boolean isActive=false;
        double maxOverShoot = 0;
        boolean hasReached = false;
        long timeToGetTo = 0;
        double error = 0;
        double derivative;
        double testTimer = 10; //seconds
        long timerStart=0;
        double curentPos;
        double motorPower=0;
        double lastError=0;
        ElapsedTime timer=new ElapsedTime();
        double pid=0;
        long startingTime=0,timeToReach;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        DcMotor outakeLeftMotor = hardwareMap.dcMotor.get("outakeleftmotor");
        DcMotor outakeRightMotor = hardwareMap.dcMotor.get("outakerightmotor");
        //DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");


        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        if(reverse) motor.setDirection(DcMotorSimple.Direction.REVERSE);


        outakeLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outakeRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();
        dashboardTelemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.

        double outakeMotorPower =0;
        double intakeMotorPower =0;
        intakeTargetPos = 0;
        outakeTargetPos = 0;
        //Servo tester = hardwareMap.get(Servo.class, "tester");
        ControlMotor intakeControlMotor = new ControlMotor();
        ControlMotor outakeControlMotor = new ControlMotor();
        resetStuff();
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            curentPos = motor.getCurrentPosition();

            if(startingTime==0)
                startingTime = System.currentTimeMillis();



            //checking if has reached
            if(curentPos>targetPosition) {
                hasReached = true;
                timeToReach = System.currentTimeMillis()-startingTime;
                startingTime=0;
            }

            // OverShoot Measurment
            maxOverShoot = Math.max(maxOverShoot,curentPos);

            //RESET
            if(gamepad1.a){
                autoKp=0;
                autoKd=0;
            }

            isActive = false;

            if(timerStart + testTimer*1000 > System.currentTimeMillis()){

                error= targetPosition - curentPos;
                derivative=(error-lastError) / timer.seconds();


                lastError = error;
                timer.reset();


                pid = error*autoKp+derivative*autoKd;

                motorPower = pid;

                isActive = true;
            }
            motor.setPower(motorPower);


            //MODIFY PIDS
            if(!isActive){
                timerStart = System.currentTimeMillis();

                lastOverShoot = overShoot;
                overShoot = maxOverShoot - targetPosition;

                //if(lastOverShoot-overShoot)


            }


            //telemetry
            telemetry.addData("motor pos ", motor.getCurrentPosition());
            telemetry.addData("motor power", motorPower);
            telemetry.addData("target POS",targetPosition);
            dashboardTelemetry.addData("target POS",targetPosition);
            dashboardTelemetry.addData("motor pos ", motor.getCurrentPosition());
            dashboardTelemetry.addData("motor power", motorPower);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            FtcDashboard.getInstance().startCameraStream(portal, 10);
            dashboardTelemetry.update();
            telemetry.update();
        }

    }
}