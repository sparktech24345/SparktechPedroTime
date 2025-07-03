package pedroPathing.Autos.Linear;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import pedroPathing.ClassWithStates;

@Config
@Autonomous(name = "RedBasketExtendAutoLinear", group = "Examples")
public class RedBasketExtendAutoLinear extends AutoForBasketMainFileLinear {
    public RedBasketExtendAutoLinear() {
        super(ClassWithStates.colorList.red);
    }
}
