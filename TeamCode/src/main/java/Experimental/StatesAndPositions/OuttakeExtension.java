package Experimental.StatesAndPositions;

public enum OuttakeExtension {
    IGNORE(0),
    UndefinedPos(0),
    MaxMotorPos(2100),
    MaxLowBasketPos(800),
    SpecimenHangPos(1100),
    AutoSpecimenHangPos(950),
    WallPickupPos(710),
    StandbyPos(1000),
    TransferPos(0),
    ParkedPos(655);
    private double value = 0;
    OuttakeExtension(double value) { this.value = value; }
    public void set(double value) { this.value = value; }
    public double get() { return this.value; }
}
