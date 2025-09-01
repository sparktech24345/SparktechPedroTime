package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import java.util.ArrayDeque;
import java.util.Queue;

import Experimental.HelperClasses.Actions.Action;

public class StateQueuer {

    private final Queue<Action> actionQueue = new ArrayDeque();
    boolean prevDone = true;

    public int getLen() {
        return actionQueue.size();
    }

    public void addAction(Action action) {
        actionQueue.add(action);
    }

    public boolean isEmpty() { return actionQueue.isEmpty(); }

    public void update() {
        prevDone = true;
        for (Action action : actionQueue) {
            if (action.started && !action.done)
                action.update();
            if (!action.started && (prevDone || !action.waitForPrevious)) {
                action.execute();
            }
            if (action.waitForPrevious && !prevDone) {
                break;
            }
            prevDone = action.done;
        }
    }
}
