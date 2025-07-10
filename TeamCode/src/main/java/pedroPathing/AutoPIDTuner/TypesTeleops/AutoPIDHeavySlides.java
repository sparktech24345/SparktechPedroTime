package pedroPathing.AutoPIDTuner.TypesTeleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.AutoPIDTuner.TrainAutoPIDBaseTeleopHandler;
import pedroPathing.AutoPIDTuner.TunningTypes;

@TeleOp(name = "AutoPIDHeavySlides", group = "AutoPIDS")
public class AutoPIDHeavySlides extends TrainAutoPIDBaseTeleopHandler {
    public AutoPIDHeavySlides(){
        super(TunningTypes.heavySlidesTuning);
    }
}
