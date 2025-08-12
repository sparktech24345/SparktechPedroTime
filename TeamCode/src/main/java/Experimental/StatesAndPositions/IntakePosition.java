package Experimental.StatesAndPositions;

public enum IntakePosition {
    UndefinedPos(0),
    PickupSamplePos(207),
    SpitOutPos(15),
    TransferPos(153);
    private double value = 0;
    IntakePosition(double value) { this.value = value; }
    public void set(double value) { this.value = value; }
    public double get() { return this.value; }
}
