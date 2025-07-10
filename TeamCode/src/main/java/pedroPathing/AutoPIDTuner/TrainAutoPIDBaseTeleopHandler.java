package pedroPathing.AutoPIDTuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.ClassWithStates;


@Config
public class TrainAutoPIDBaseTeleopHandler extends LinearOpMode {

    AutoPIDTuner autoPIDTuner;
    TunningTypes currentTuningType;
    public TrainAutoPIDBaseTeleopHandler(TunningTypes currentTuningType){
        this.currentTuningType = currentTuningType;
    }
    @Override
    public void runOpMode() throws InterruptedException {


        autoPIDTuner = new AutoPIDTuner(hardwareMap);

        waitForStart();

        if (isStopRequested()){
            return;
        }



    }









}
