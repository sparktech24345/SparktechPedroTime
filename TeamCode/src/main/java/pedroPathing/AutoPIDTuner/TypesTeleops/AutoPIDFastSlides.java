package pedroPathing.AutoPIDTuner.TypesTeleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.AutoPIDTuner.TrainAutoPIDBaseTeleopHandler;
import pedroPathing.AutoPIDTuner.TunningTypes;

@TeleOp(name = "AutoPIDFastSlides Recommended", group = "AutoPIDS")
public class AutoPIDFastSlides extends TrainAutoPIDBaseTeleopHandler {
    public AutoPIDFastSlides(){
        super(TunningTypes.fastSlidesTuning);
    }
}
