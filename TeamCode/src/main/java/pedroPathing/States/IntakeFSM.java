package pedroPathing.States;

interface State {
    void execute();
    //made by ai duuh
}

// Define the TransitionState class


class TransitionState implements State {
    private State nextState;

    public TransitionState(State nextState) {
        this.nextState = nextState;
    }

    @Override
    public void execute() {
        // Perform any transition logic here
        //System.out.println("Transitioning to the next state...");

        nextState.execute();
    }
}


// Define the Intake FSM
public class IntakeFSM {
    public State currentStateIntake;

    public IntakeFSM(State initialState) {
        this.currentStateIntake = initialState;
    }

    public void setState(State newState) {
        TransitionState transition = new TransitionState(newState);
        transition.execute();
        this.currentStateIntake = newState;
    }

    public void executeCurrentState() {
        currentStateIntake.execute();
    }
}

///DEMONSTRATION
/*
        // Initialize Outtake states
        State outtakeSpecimen = new OuttakeStateSpecimen();
        State outtakeBasket = new OuttakeStateBasket();
        State outtakeSamplePickUp = new OuttakeStateSamplePickUp();
        State outtakeStandbyDown = new OuttakeStateStandbyDownWithSample();
        State outtakeStandby = new OuttakeStateStandbyWithSample();

        // Initialize Intake states
        State intakeRetracted = new IntakeStateRetracted();
        State intakeExtended = new IntakeStateExtended();

        // Create the Outtake FSM with the initial state
        OuttakeFSM outtakeFSM = new OuttakeFSM(outtakeSpecimen);
        outtakeFSM.executeCurrentState();

        // Create the Intake FSM with the initial state
        IntakeFSM intakeFSM = new IntakeFSM(intakeRetracted);
        intakeFSM.executeCurrentState();

        // Transition in Outtake FSM
        outtakeFSM.setState(outtakeStandbyDown);
        outtakeFSM.executeCurrentState();

        // Transition in Intake FSM
        intakeFSM.setState(intakeExtended);
        intakeFSM.executeCurrentState();

        // Example of transitioning from Outtake to Intake
        System.out.println("Transitioning from Outtake to Intake...");
        outtakeFSM.setState(outtakeStandby);
        outtakeFSM.executeCurrentState();
        intakeFSM.setState(intakeRetracted);
        intakeFSM.executeCurrentState();
 */
