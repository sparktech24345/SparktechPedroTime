package Experimental.HelperClasses;

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
            action.update(prevDone);
            prevDone = action.finished();
        }
        while (!actionQueue.isEmpty()) {
            if (actionQueue.peek().finished()) actionQueue.poll();
            else break;
        }
    }
}
