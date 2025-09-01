package Experimental.HelperClasses;

import androidx.annotation.Nullable;

import java.util.Optional;

import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;

public enum RobotState {
    StandbyState(
        IntakeExtension.Retracted,
        IntakePosition.TransferPos,
        OuttakeExtension.SpecimenHangPos,
        OuttakeArmPosition.StandbyPos,
        OuttakeClawPosition.RetractedPos
    ),
    StartState(
        IntakeExtension.Retracted,
        IntakePosition.SpitOutPos,
        OuttakeExtension.TransferPos,
        OuttakeArmPosition.StandbyPos,
        OuttakeClawPosition.RetractedPos
    ),
    TransferState(
        IntakeExtension.Retracted,
        IntakePosition.TransferPos,
        OuttakeExtension.TransferPos,
        OuttakeArmPosition.TransferPos
    ),

    HighBasket(
            OuttakeArmPosition.BasketPos,
            OuttakeExtension.MaxMotorPos
    ),

    CloseClaw(
        OuttakeClawPosition.RetractedPos
    ),

    OpenClaw(
        OuttakeClawPosition.ExtraExtendedPos
    ),

    SamplePickup(
        IntakePosition.PickupSamplePos
    ),

    IntakeExtension1(
            IntakeExtension.Extended1
    ),

    IntakeExtension2(
            IntakeExtension.Extended2
    ),

    IntakeExtension3(
            IntakeExtension.Extended3
    ),

    IntakeExtension4(
            IntakeExtension.Extended4
    ),

    IntakeRetracted(
            IntakeExtension.Retracted
    ),

    SpecimenHang(
            OuttakeExtension.SpecimenHangPos,
            OuttakeArmPosition.HighRungPos
    )

    ;

    public IntakeExtension intakeExtension = IntakeExtension.IGNORE;
    public IntakePosition intakePosition = IntakePosition.IGNORE;
    public OuttakeExtension outtakeExtension = OuttakeExtension.IGNORE;
    public OuttakeArmPosition outtakeArmPosition = OuttakeArmPosition.IGNORE;
    public OuttakeClawPosition outtakeClawPosition = OuttakeClawPosition.IGNORE;

    <T> void ParseComponent(T component) {
        if (component instanceof IntakeExtension) {
            this.intakeExtension = (IntakeExtension) component;
        }
        if (component instanceof IntakePosition) {
            this.intakePosition = (IntakePosition) component;
        }
        if (component instanceof OuttakeExtension) {
            this.outtakeExtension = (OuttakeExtension) component;
        }
        if (component instanceof OuttakeArmPosition) {
            this.outtakeArmPosition = (OuttakeArmPosition) component;
        }
        if (component instanceof OuttakeClawPosition) {
            this.outtakeClawPosition = (OuttakeClawPosition) component;
        }
    }

    <T> RobotState(T component) {
        ParseComponent(component);
    }

    <T1, T2> RobotState(T1 c1, T2 c2) {
        ParseComponent(c1);
        ParseComponent(c2);
    }

    <T1, T2, T3> RobotState(T1 c1, T2 c2, T3 c3) {
        ParseComponent(c1);
        ParseComponent(c2);
        ParseComponent(c3);
    }

    <T1, T2, T3, T4> RobotState(T1 c1, T2 c2, T3 c3, T4 c4) {
        ParseComponent(c1);
        ParseComponent(c2);
        ParseComponent(c3);
        ParseComponent(c4);
    }

    <T1, T2, T3, T4, T5> RobotState(T1 c1, T2 c2, T3 c3, T4 c4, T5 c5) {
        ParseComponent(c1);
        ParseComponent(c2);
        ParseComponent(c3);
        ParseComponent(c4);
        ParseComponent(c5);
    }
}
