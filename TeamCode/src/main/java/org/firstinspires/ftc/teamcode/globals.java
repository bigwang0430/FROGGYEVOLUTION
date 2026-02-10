package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class globals {
    @Config
    public static class launcher {


        public static float p =0.00005F; //0.001
        public static float i = 0.1F;
        public static float d = 0F;
        public static float ks = 0.001F; //0.0000216
        public static float kv = 0.00018F; //0.000000120871
        public static float squP = 0F;
        public static float squI = 0F;
        public static float squD = 0F;
        public static float squKs = 0F;
        public static float squKv = 0F;
        public static float velTime = 1.2F;
        public static float turretOffset = 5F;
    }

    @Config
    public static class testing {
        public static float targetRPM = 3000F;
        public static float hoodAng = 60F;
        public static boolean SquidOn = false;
        public static boolean launcherOn = false;
        public static float turretAng = 0F;
    }
    @Config
    public static class kalman {

        // Starting uncertainty (covariance) in odometry estimate, in inches^2
        public static double pX0 = 0.001;
        public static double pY0 = 0.001;

        // How much uncertainty (covariance) in odometry you add each update loop, in inches^2 per loop
        public static double qX = 1;
        public static double qY = 1;

        // Camera measurement noise (variance), in inches^2
        public static double rX = 4;
        public static double rY = 4;
    }

    @Config
    public static class gate {
        public static float close = 0.25F; //
        public static float open = 0.5F;
    }
}
