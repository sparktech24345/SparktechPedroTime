package Experimental.HelperClasses;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Objects;
import java.util.Queue;

import Experimental.HelperClasses.Actions.Action;
import Experimental.HelperClasses.Actions.StateAction;

public class StateQueuer {

    private final Queue<Action> actionQueue = new ArrayDeque();
    boolean prevDone = true;

    public int getLen() {
        return actionQueue.size();
    }

    public StateQueuer addAction(Action action) {
        actionQueue.add(action);
        return this;
    }

    /**
     *
     */
    public StateQueuer addAction(Action... actions) {
        Collections.addAll(actionQueue, actions);
        return this;
    }

    public boolean isEmpty() { return actionQueue.isEmpty(); }

    public void update() {
        prevDone = true;
        for (Action action : actionQueue) {
            action.update(prevDone);
            prevDone = action.finished();
        }
        while (!actionQueue.isEmpty()) {
            if (actionQueue.peek().finished()) actionQueue.poll();
            else break;
        }
    }
}
