package Experimental.StatesAndPositions;

public enum OuttakeClawPosition {
    IGNORE(0),
    UndefinedPos(0),
    ExtendedPos(128),
    ExtraExtendedPos(208),
    RetractedPos(35);
    private double value = 0;
    OuttakeClawPosition(double value) { this.value = value; }
    public void set(double value) { this.value = value; }
    public double get() { return this.value; }
}
