package pedroPathing.AutoPIDTuner.TypesTeleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.AutoPIDTuner.TrainAutoPIDBaseTeleopHandler;
import pedroPathing.AutoPIDTuner.TunningTypes;

@TeleOp(name = "AutoPIDArm", group = "AutoPIDS")
public class AutoPIDArm extends TrainAutoPIDBaseTeleopHandler {
    public AutoPIDArm(){
        super(TunningTypes.rotatingArmTuning);
    }
}
