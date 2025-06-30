package pedroPathing.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import pedroPathing.ClassWithStates;

@Config
@Autonomous(name = "BlueBasketExtendAuto", group = "Examples")
public class BlueBasketExtendAuto extends AutoForBasketMainFile {
    public BlueBasketExtendAuto() {
        super(ClassWithStates.colorList.blue);
    }
}
