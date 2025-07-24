package Experimental.StatesAndPositions;

public enum IntakeExtension {
    UndefinedPos(0, 0),
    IGNORE(0, 0),
    Extended4(585, 8),
    Extended3(377, 6),
    Extended2(245, 4),
    Extended1(112, 2),
    Retracted(0, 0);
    
    private double value = 0;
    private double gravitySubtractor = 0;
    IntakeExtension(double init_value, double gravity_subtractor) {
        this.value = init_value;
        this.gravitySubtractor = gravity_subtractor; 
    }
    public double get() { return this.value; }
    public void set(double val) { this.value = val; }
}
