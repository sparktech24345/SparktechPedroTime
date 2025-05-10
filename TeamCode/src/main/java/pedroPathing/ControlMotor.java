package pedroPathing;
import com.qualcomm.robotcore.util.ElapsedTime;
@com.acmerobotics.dashboard.config.Config
public class ControlMotor {
    public static double integralSum =0;
    public static double kpIntake =0.0090; //old is 90
    public static double kdIntake =0.0002;
    //double ki=0.02;
    //double kf=0;

    public static double kpUppy=0.0030; //old is 0.0030
    public static double kdUppy=0.0001;
    ElapsedTime timer=new ElapsedTime();

    public double getLastError() {
        return lastError;
    }

    private double lastError=0;

    public double PIDControl(double targetPosition, double curentPosition){
        double error= targetPosition - curentPosition;
        //integralSum +=error * timer.seconds();
        double derivative=(error-lastError) / timer.seconds();

        lastError = error;
        timer.reset();
        double pid = (error* kpIntake +derivative* kdIntake);
        if(pid<0) pid*=1.2;
        /*if(pid<0 && curentPosition<40) pid = -0.3;
        if(pid<0 && curentPosition<10) pid = -0.25;
        if(pid<0 && curentPosition<=3) pid = 0;//*/
        return pid;

        //return (error*kp+derivative*kd) * (((lastError < 0 && lastError > -50) || (lastError  < -415 && lastError > -450))  ? 2 : 1);
        //return ((error*kp)+(derivative*kd)+(integralSum *ki)+(targetPosition*kf));
    }


    public double PIDControlUppy(double targetPosition, double curentPosition){
        double error= targetPosition - curentPosition;
        //integralSum +=error * timer.seconds();
        double derivative=(error-lastError) / timer.seconds();

        lastError = error;
        timer.reset();
        double pid = error*kpUppy+derivative*kdUppy;
        /*TODO: SA IL FACI MAI FRUMOS, L-AM SCRIS ASA LA GRABA SI SA INVERSEZI SI TU
        NEGATIVUL CU POZITIVUL CA E CONTRA INTUITIV ACUM ~vlad*/
        /*if (pid > 0.01 && curentPosition < -2) {
            if (curentPosition < -25) {
                pid = 0.002;
            } else {
                pid = 0.3;
            }
        }//*/

        if(targetPosition==0 && curentPosition >= -350) pid *=0.8;
        if(targetPosition > -900 && curentPosition >= -800) pid*=1.6;
        if(curentPosition > -20 && targetPosition==0) pid = 0;
        if(curentPosition > -200 && targetPosition==0) pid *= 2;
        if(targetPosition < -2300 && curentPosition >=-200) pid*=2;
        return pid;

    }
}


