package Experimental.HelperClasses;

public class RobotState {
    public float IntakeOrientation = 0;
    public float IntakeExtension = 0;
    public float OuttakeExtension = 0;
    public float OuttakeArmPos = 0;
    public float OuttakeClawPos = 0;

    public RobotState() {}
    public RobotState(float intake_orientation, float intake_extension, float outtake_extension, float outtake_arm_pos, float outtake_claw_pos) {
        IntakeOrientation = intake_orientation;
        IntakeExtension = intake_extension;
        OuttakeExtension = outtake_extension;
        OuttakeArmPos = outtake_arm_pos;
        OuttakeClawPos = outtake_claw_pos;
    }
}
