package pedroPathing.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import pedroPathing.ClassWithStates;

@Config
@Autonomous(name = "BlueBasketExtendAuto", group = "Examples")
@Disabled
public class BlueBasketExtendAuto extends AutoForBasketMainFile {
    public BlueBasketExtendAuto() {
        super(ClassWithStates.colorList.blue);
    }
}
