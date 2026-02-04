package deprs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class shooterConstants {
    public static Pose blueGoal = new Pose(138, 138);
    public static double scoreHeight = 26;
    public static double scoreAngle = Math.toRadians(-30);
    public static double passThroughDist = 5;

    public static double maxHood = 10;
    public static double minHood = 10;

    public static double maxRPM = 5500;
    public static double minRPM = 0;
}
