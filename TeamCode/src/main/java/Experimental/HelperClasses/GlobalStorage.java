package Experimental.HelperClasses;

import Experimental.StatesAndPositions.IntakeExtension;
import Experimental.StatesAndPositions.IntakePosition;
import Experimental.StatesAndPositions.OuttakeArmPosition;
import Experimental.StatesAndPositions.OuttakeClawPosition;
import Experimental.StatesAndPositions.OuttakeExtension;

public class GlobalStorage {
    public static float[] IntakePositions = new float[IntakePosition.values().length];
    public static float[] IntakeExtensions = new float[IntakeExtension.values().length];
    public static float[] OuttakeExtensions = new float[OuttakeExtension.values().length];
    public static float[] OuttakeArmPositions = new float[OuttakeArmPosition.values().length];
    public static float[] OuttakeClawPositions = new float[OuttakeClawPosition.values().length];

    public static void InitAllPositions() {
        IntakePositions[IntakePosition.PickupSamplePos.ordinal()] = 181;
        IntakePositions[IntakePosition.SpitOutPos.ordinal()] = 0;
        IntakePositions[IntakePosition.TransferPos.ordinal()] = 130;

        // TO ADD INTAKE EXTENSIONS !!!
        IntakeExtensions[IntakeExtension.Retracted.ordinal()] = 0;
        IntakeExtensions[IntakeExtension.Extended1.ordinal()] = 0;
        IntakeExtensions[IntakeExtension.Extended2.ordinal()] = 0;
        IntakeExtensions[IntakeExtension.Extended3.ordinal()] = 0;
        IntakeExtensions[IntakeExtension.Extended4.ordinal()] = 0;

        OuttakeExtensions[OuttakeExtension.MaxMotorPos.ordinal()] = 2100;
        OuttakeExtensions[OuttakeExtension.MaxLowBasketPos.ordinal()] = 800;
        OuttakeExtensions[OuttakeExtension.SpecimenHangPos.ordinal()] = 1100;
        OuttakeExtensions[OuttakeExtension.AutoSpecimenHangPos.ordinal()] = 950;
        OuttakeExtensions[OuttakeExtension.WallPickupPos.ordinal()] = 710;
        OuttakeExtensions[OuttakeExtension.ActualZeroPos.ordinal()] = 0;
        OuttakeExtensions[OuttakeExtension.StandbyPos.ordinal()] = 1000;
        OuttakeExtensions[OuttakeExtension.tempAPos.ordinal()] = 0;
        OuttakeExtensions[OuttakeExtension.ParkedPos.ordinal()] = 655;

        OuttakeArmPositions[OuttakeArmPosition.WallPickupPos.ordinal()] = 285;
        OuttakeArmPositions[OuttakeArmPosition.HighRungHangPos.ordinal()] = 170;
        OuttakeArmPositions[OuttakeArmPosition.BasketPos.ordinal()] = 43;
        OuttakeArmPositions[OuttakeArmPosition.TransferPos.ordinal()] = 201;
        OuttakeArmPositions[OuttakeArmPosition.StandbyPos.ordinal()] = 170;
        OuttakeArmPositions[OuttakeArmPosition.ParkedPos.ordinal()] = 164.84f; // 0.53*328 - 9

        OuttakeClawPositions[OuttakeClawPosition.ExtendedPos.ordinal()] = 128;
        OuttakeClawPositions[OuttakeClawPosition.ExtraExtendedPos.ordinal()] = 208;
        OuttakeClawPositions[OuttakeClawPosition.RetractedPos.ordinal()] = 15;
    }
}
