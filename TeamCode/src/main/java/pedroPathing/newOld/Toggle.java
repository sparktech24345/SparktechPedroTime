package pedroPathing.newOld;

public class Toggle {
    public static boolean toggled;
    public static boolean toggle_var;
    public static long  startingTime;
    public static long  startingTime2;
    public static long  startingTime3;
    public static boolean toggledButton;
    public static boolean toggledVarButton;
    public static boolean toggledButton2;
    public static boolean toggledVarButton2;
    public static boolean toggledButton3;
    public static boolean toggledVarButton3;

    public static boolean toggledUp;
    public static boolean toggle_varUp;
    public static boolean toggledLeft;
    public static boolean toggle_varLeft;
    public static boolean toggledDown;
    public static boolean toggle_varDown;
    public static boolean toggledRight;
    public static boolean toggle_varRight;


    public static boolean FirsToggle(boolean input) {
        boolean output;

        if (input) {
            if (!toggled) {
                toggle_var = !toggle_var;
                toggled = true;
            }
            output = toggle_var ? true : false;
        } else {
            toggled = false;
            output = toggle_var ? true : false;
        }

        return output;
    }

    public static double outputtoggle(boolean starter){
        if(starter){
              startingTime = System.currentTimeMillis();
        }
        if(System.currentTimeMillis() < startingTime + 300){
            return -0.7;
        }else{
            return 0;
        }
    }
    public static double outputtoggle2(boolean starter){
        if(starter){
            startingTime2 = System.currentTimeMillis();
        }
        if(System.currentTimeMillis() < startingTime2 + 1000){
            return -0.7;
        }else{
            return 0;
        }
    }
    public static double outputtoggle3(boolean starter){
        if(starter){
            startingTime3 = System.currentTimeMillis();
        }
        if(System.currentTimeMillis() < startingTime3 + 100){
            return -0.7;
        }else{
            return 0;
        }
    }

    public static int toggleButton(boolean input) {
        int output;

        if (input) {
            if (!toggledButton) {
                toggledVarButton = !toggledVarButton;
                toggledButton = true;
            }
            output = toggledVarButton ? 1 : 0;
        } else {
            toggledButton = false;
            output = toggledVarButton ? 1 : 0;
        }

        return output;
    }

    public static int toggleButton2(boolean input) {
        int output;
        if (input) {
            if (!toggledButton2) {
                toggledVarButton2 = !toggledVarButton2;
                toggledButton2 = true;
            }
            output = toggledVarButton2 ? 1 : 0;
        } else {
            toggledButton2 = false;
            output = toggledVarButton2 ? 1 : 0;
        }
        return output;
    }
    public static int toggleButton3(boolean input) {
        int output;
        if (input) {
            if (!toggledButton3) {
                toggledVarButton3 = !toggledVarButton3;
                toggledButton3 = true;
            }
            output = toggledVarButton3 ? 1 : 0;
        } else {
            toggledButton3 = false;
            output = toggledVarButton3 ? 1 : 0;
        }
        return output;
    }
    public static int toggleUp(boolean input) {
        int output;

        if (input) {
            if (!toggledUp) {
                toggle_varUp = !toggle_varUp;
                toggledUp = true;
            }
            output = toggle_varUp ? 1 : 0;
        } else {
            toggledUp = false;
            output = toggle_varUp ? 1 : 0;
        }

        return output;
    }
    public static int toggleLeft(boolean input) {
        int output;

        if (input) {
            if (!toggledLeft) {
                toggle_varLeft = !toggle_varLeft;
                toggledLeft = true;
            }
            output = toggle_varLeft ? 1 : 0;
        } else {
            toggledLeft = false;
            output = toggle_varLeft ? 1 : 0;
        }

        return output;
    }
    public static int toggleDown(boolean input) {
        int output;

        if (input) {
            if (!toggledDown) {
                toggle_varDown = !toggle_varDown;
                toggledDown = true;
            }
            output = toggle_varDown ? 1 : 0;
        } else {
            toggledDown = false;
            output = toggle_varDown ? 1 : 0;
        }

        return output;
    }
    public static int toggleRight(boolean input) {
        int output;

        if (input) {
            if (!toggledRight) {
                toggle_varRight = !toggle_varRight;
                toggledRight = true;
            }
            output = toggle_varRight ? 1 : 0;
        } else {
            toggledRight = false;
            output = toggle_varRight ? 1 : 0;
        }

        return output;
    }
}
