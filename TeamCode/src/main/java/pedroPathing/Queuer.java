package pedroPathing;

import java.util.LinkedList;
import java.util.Queue;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Queuer {

    //step
    public static class Step {
        private boolean condition;  //predicat ish
        private Runnable runny;    //run

        //this step constructor thingy
        public Step(boolean condition, Runnable action) {
            this.condition = condition;
            this.runny = action;
        }


        //check condition
        public boolean canRun() {
            return condition; //return true if condition is met
        }

        //run
        public void run() {
                runny.run();
        }
    }




    //queue made with linked list from cpp
    private Queue<Step> steps = new LinkedList<>();



    //add step to list
    public void addStep(Step step) {
        steps.offer(step);
    }



    //try executing step
    public void updateSteps(Telemetry telemetry) {
        if(!steps.isEmpty()) {
            Step firstStep = steps.peek();// go to first step

            if (firstStep.canRun()) {
                steps.poll().run();  //will run step if condition true
            } else {
                telemetry.addData("trying to execute a step",true);
            }
        }
        else telemetry.addData("no steps left",true);
    }


    //step pattern

    public static Step steper = new Step(false, new Runnable() {
        public void run() {
            boolean isItAStep = true;
        }
    });


    //method pattern
    /*
    public static void StepsTime (){
        queuer.addStep(steper);
        queuer.addStep(steper);
        queuer.addStep(steper);
    }
     */
}
