package Experimental.HelperClasses.Actions;

import java.util.function.BooleanSupplier;

import Experimental.HelperClasses.Checkable;

public class ConditionChecker extends Action {

    public ConditionChecker(BooleanSupplier condition) {
        this.ExecutionCondition = condition;
    }
}
