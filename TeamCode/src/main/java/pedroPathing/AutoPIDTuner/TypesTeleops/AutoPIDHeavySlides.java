package pedroPathing.AutoPIDTuner.TypesTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.AutoPIDTuner.TrainAutoPIDBaseTeleopHandler;
import pedroPathing.AutoPIDTuner.TunningTypes;

@TeleOp(name = "AutoPIDHeavySlides", group = "AutoPIDS")
@Disabled
public class AutoPIDHeavySlides extends TrainAutoPIDBaseTeleopHandler {
    public AutoPIDHeavySlides(){
        super(TunningTypes.heavySlidesTuning);
    }
}
