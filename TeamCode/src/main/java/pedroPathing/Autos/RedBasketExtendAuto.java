package pedroPathing.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import pedroPathing.ClassWithStates;

@Config
@Autonomous(name = "RedBasketExtendAuto", group = "Examples")
public class RedBasketExtendAuto extends AutoForBasketMainFile {
    public RedBasketExtendAuto() {
        super(ClassWithStates.colorList.red);
    }
}
