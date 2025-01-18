package pedroPathing.States;

import static pedroPathing.States.PositionStorage.*;

public class OuttakeStateBasket implements State {
    @Override
    public void execute() {
        //System.out.println("Executing OuttakeStateBasket...");
        // Set servo and motor positions for this state
        stateStringOutake = "OuttakeStateBasket";
        isOuttakeStateBascket = true;
        wasOuttakeStateBascket = true;
        if(outakeSampleServoPosition != servoextended)
         outakeSampleServoPosition=outakeSampleRetracted;
        //outakeArmServoPosition = 150; //185
        //if(armServoPos>=80)
        outakeArmServoPosition = 285;
        intakeMotorPickUpPower =0;
        outakeTargetPos = -2800;
    }
}
