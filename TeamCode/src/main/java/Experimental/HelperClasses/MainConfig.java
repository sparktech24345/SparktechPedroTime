package Experimental.HelperClasses;

import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;
import Experimental.StatesAndPositions.RobotStates;
import static Experimental.HelperClasses.GlobalStorage.*;

public class MainConfig {
    public static RobotState[] AllStates = new RobotState[RobotStates.values().length];
    public static RobotStates CurrentState = RobotStates.StartState;

    public static void InitAllStates() {
        AllStates[RobotStates.StartState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.SampleTransferState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.TransferPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.ActualZeroPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.TransferPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.ExtendedPos.ordinal()]
        );


        AllStates[RobotStates.SamplePickupState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.PickupSamplePos.ordinal()],
                IntakeExtensions[IntakeExtension.Extended4.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.WallPickupReadyState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.TransferPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.WallPickupPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.WallPickupPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.ExtraExtendedPos.ordinal()]
        );


        AllStates[RobotStates.WallPickedUpState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.TransferPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.WallPickupPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.WallPickupPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.HighBasketScoreState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.LowBasketScoreState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.SpecimenHangState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.HighRungReadyState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );


        AllStates[RobotStates.HighRungHangState.ordinal()] = new RobotState(
                IntakePositions[IntakePosition.SpitOutPos.ordinal()],
                IntakeExtensions[IntakeExtension.Retracted.ordinal()],
                OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()],
                OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()],
                OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()]
        );
    }
}
