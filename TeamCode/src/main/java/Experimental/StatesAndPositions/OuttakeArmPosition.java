package Experimental.StatesAndPositions;

public enum OuttakeArmPosition {
    IGNORE(0),
    UndefinedPos(0),
    WallPickupPos(285),
    HighRungPos(170),
    BasketPos(43),
    TransferPos(201),
    StandbyPos(170),
    ParkedPos(164.84f);
    private double value = 0;
    OuttakeArmPosition(double value) { this.value = value; }
    public void set(double value) { this.value = value; }
    public double get() { return this.value; }
}
