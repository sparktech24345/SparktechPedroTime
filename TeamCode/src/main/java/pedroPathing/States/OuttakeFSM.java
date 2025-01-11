package pedroPathing.States;

// Define the Outtake FSM
public class OuttakeFSM {
    public State currentStateOutake;

    public OuttakeFSM(State initialState) {
        this.currentStateOutake = initialState;
    }

    public void setState(State newState) {
        TransitionState transition = new TransitionState(newState);
        transition.execute();
        this.currentStateOutake = newState;
    }

    public void executeCurrentState() {
        currentStateOutake.execute();
    }
}
