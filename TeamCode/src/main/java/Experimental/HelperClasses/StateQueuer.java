package Experimental.HelperClasses;

import static Experimental.HelperClasses.GlobalStorage.*;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Objects;
import java.util.Queue;

import Experimental.HelperClasses.Actions.Action;

public class StateQueuer {

    private final Queue<Action> actionQueue = new ArrayDeque();
    private boolean prevDone;

    public int getLen() {
        return actionQueue.size();
    }

    public StateQueuer addAction(Action action) {
        actionQueue.add(action);
        return this;
    }

    public boolean isEmpty() { return actionQueue.isEmpty(); }

    public void update() {
        prevDone = true;
        for (Action action : actionQueue) {
            action.update(prevDone);
            if (action.waits() && !prevDone) break;
            prevDone = action.finished();
        }
        while (!actionQueue.isEmpty()) {
            if (actionQueue.peek().finished()) actionQueue.poll();
            else break;
        }
    }
}





















//          if (actionQueue.peek().finished()) actionQueue.poll();
