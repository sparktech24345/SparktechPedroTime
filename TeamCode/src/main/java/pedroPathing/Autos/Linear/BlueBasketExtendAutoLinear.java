package pedroPathing.Autos.Linear;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import pedroPathing.ClassWithStates;

@Config
@Autonomous(name = "BlueBasketExtendAutoLinear", group = "Examples")
public class BlueBasketExtendAutoLinear extends AutoForBasketMainFileLinear {
    public BlueBasketExtendAutoLinear() {
        super(ClassWithStates.colorList.blue);
    }
}
