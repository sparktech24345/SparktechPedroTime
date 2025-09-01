package Experimental.HelperClasses;

import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;

public enum RobotState {
    StandbyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.TransferPos,
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
    SampleTransferReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.TransferPos,
            OuttakeArmPosition.TransferPos,
            OuttakeClawPosition.ExtendedPos
    ),
    SampleTransferDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.TransferPos,
            OuttakeArmPosition.TransferPos,
            OuttakeClawPosition.RetractedPos
    ),
    SamplePickupReadyState(
            IntakeExtension.IGNORE,
            IntakePosition.PickupSamplePos,
            OuttakeExtension.ParkedPos,
            OuttakeArmPosition.StandbyPos,
            OuttakeClawPosition.RetractedPos
    ),
    SamplePickupDoneState(
            IntakeExtension.IGNORE,
            IntakePosition.TransferPos,
            OuttakeExtension.ParkedPos,
            OuttakeArmPosition.TransferPos,
            OuttakeClawPosition.RetractedPos
    ),
    WallPickupReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.WallPickupPos,
            OuttakeArmPosition.WallPickupPos,
            OuttakeClawPosition.ExtraExtendedPos
    ),
    WallPickupDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.WallPickupPos,
            OuttakeArmPosition.WallPickupPos,
            OuttakeClawPosition.RetractedPos
    ),
    HighBasketScoreReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.MaxMotorPos,
            OuttakeArmPosition.BasketPos,
            OuttakeClawPosition.RetractedPos
    ),
    HighBasketScoreDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.MaxMotorPos,
            OuttakeArmPosition.BasketPos,
            OuttakeClawPosition.ExtendedPos
    ),
    LowBasketScoreReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.MaxLowBasketPos,
            OuttakeArmPosition.BasketPos,
            OuttakeClawPosition.RetractedPos
    ),
    LowBasketScoreDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.MaxLowBasketPos,
            OuttakeArmPosition.BasketPos,
            OuttakeClawPosition.ExtendedPos
    ),
    SpecimenHangReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.SpecimenHangPos,
            OuttakeArmPosition.HighRungPos,
            OuttakeClawPosition.RetractedPos
    ),
    SpecimenHangDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.SpecimenHangPos,
            OuttakeArmPosition.HighRungPos,
            OuttakeClawPosition.ExtendedPos
    ),
    HangReadyState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.MaxMotorPos,
            OuttakeArmPosition.StandbyPos,
            OuttakeClawPosition.RetractedPos
    ),
    HangDoneState(
            IntakeExtension.Retracted,
            IntakePosition.TransferPos,
            OuttakeExtension.SpecimenHangPos,
            OuttakeArmPosition.StandbyPos,
            OuttakeClawPosition.RetractedPos
    );

    public final IntakeExtension intakeExtension;
    public final IntakePosition intakePosition;
    public final OuttakeExtension outtakeExtension;
    public final OuttakeArmPosition outtakeArmPosition;
    public final OuttakeClawPosition outtakeClawPosition;

    RobotState(
            IntakeExtension intakeExtension,
            IntakePosition intakePosition,
            OuttakeExtension outtakeExtension,
            OuttakeArmPosition outtakeArmPosition,
            OuttakeClawPosition outtakeClawPosition
    )
    {
        this.intakeExtension = intakeExtension;
        this.intakePosition = intakePosition;
        this.outtakeExtension = outtakeExtension;
        this.outtakeArmPosition = outtakeArmPosition;
        this.outtakeClawPosition = outtakeClawPosition;
    }
}
