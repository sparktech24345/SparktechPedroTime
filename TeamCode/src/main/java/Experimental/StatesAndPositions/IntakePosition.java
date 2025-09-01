package Experimental.StatesAndPositions;

public enum IntakePosition {
    IGNORE(0),
    UndefinedPos(0),
    PickupSamplePos(211),
    SpitOutPos(15),
    TransferPos(153);
    private double value = 0;
    IntakePosition(double value) { this.value = value; }
    public void set(double value) { this.value = value; }
    public double get() { return this.value; }
}
