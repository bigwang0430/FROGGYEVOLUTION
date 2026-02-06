package org.firstinspires.ftc.teamcode.deprs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class shooterConstants {
    public static Pose blueGoal = new Pose(138, 138);
    public static double scoreHeight = 26;
    public static double scoreAngle = Math.toRadians(-30);
    public static double passThroughDist = 5;

    public static double maxHood = 0;
    public static double minHood = 200;

    public static double maxRPM = 4000;
    public static double minRPM = 0;
}
