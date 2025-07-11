package pedroPathing.AutoPIDTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.ClassWithStates;


@Config
public class TrainAutoPIDBaseTeleopHandler extends LinearOpMode {

    AutoPIDTuner autoPIDTuner;
    TunningTypes currentTuningType;
    MultipleTelemetry tel;
    public TrainAutoPIDBaseTeleopHandler(TunningTypes currentTuningType){
        this.currentTuningType = currentTuningType;
    }
    @Override
    public void runOpMode() throws InterruptedException {

        tel =  new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        autoPIDTuner = new AutoPIDTuner(hardwareMap, this,tel);

        tel.addLine("Auto PID tuner made by team Sparktech 24345");
        tel.addLine("Please use this at the voltage you want it to work perfectly or a bit lower");
        tel.addLine("For accurate results please run this 3-5 times at slightly different voltages and average out");
        tel.addLine("Personal recommendation: voltage range between 13.1V and 13.8V");
        tel.update();

        waitForStart();

        if (isStopRequested()){
            return;
        }

        autoPIDTuner.setServos();
        autoPIDTuner.fullPowerRun();
        autoPIDTuner.returnMotor();

        if(currentTuningType == TunningTypes.fastSlidesTuning) autoPIDTuner.runOscillationAutotune();
        if(currentTuningType == TunningTypes.heavySlidesTuning) autoPIDTuner.runOscillationAutotuneBrutal();


    }









}
