package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class globals {
    @Config
    public static class launcher {
        public static float targetRPM = 3000F;
        public static boolean SquidOn = false;
        public static boolean launcherOn = false;
        public static float p =0.0009F; //0.001
        public static float i = 0.2F;
        public static float d = 0F;
        public static float ks = 0.0000216F; //0.0000216
        public static float kv = 0.00018F; //0.000000120871
        public static float squP = 0F;
        public static float squI = 0F;
        public static float squD = 0F;
        public static float squKs = 0F;
        public static float squKv = 0F;
    }
}
