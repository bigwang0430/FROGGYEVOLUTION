package org.firstinspires.ftc.teamcode.deprs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import java.util.Objects;

@TeleOp (name = "testTele")
public class    testTele extends OpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(18, 18);
    private Motor launch1, launch2, intake, transfer;
    private Follower follower;
    private GamepadEx g1;

    private int lastPosition;
    private double lastTime;

    private double launchpower;
    private double RPM, previousRPM;
    private ServoEx hood, turret;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private boolean launch, zoom;
    private double targetRPM, hoodAngle, tangentVelocity, normalVelocity;
    private String robotLocation;
    @Override
    public void init() {

        turret = new ServoEx(hardwareMap, "t2", 360, AngleUnit.DEGREES);

        launch1 = new Motor(hardwareMap, "l1", 28, 6000);
        launch2 = new Motor(hardwareMap, "l2", 28, 6000);
        launch1.setRunMode(Motor.RunMode.RawPower);
        launch2.setRunMode(Motor.RunMode.RawPower);
        launch2.setInverted(true);
        launch1.setInverted(false);
        launch1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launch2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        intake = new Motor(hardwareMap, "intake");
        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(true);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300, AngleUnit.DEGREES);

        g1 = new GamepadEx(gamepad1);
        launchPIDF.setTolerance(50);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(16, 75, Math.PI/2)); //TEMPORARY
    }

    @Override
    public void loop() {
        if (previousRPM - RPM > 300) {
            double dip = previousRPM;
            telemetry.addData("niga", dip);
        }


        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        follower.update();
        RPM();
        if (g1.getButton(GamepadKeys.Button.CROSS) && targetRPM > 0) {
            hood.set(hoodClamp(hoodAngle));
            launchPIDF.setSetPoint(targetRPM);
            launchpower = launchPIDF.calculate(RPM);
            launch = true;
            if (RPM < 400) {
                launch1.set(0.4);
                launch2.set(0.4);
            } else {

                launch1.set(launchpower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                launch2.set(launchpower + globals.launcher.kv * targetRPM + globals.launcher.ks);
            }


            if (launchPIDF.atSetPoint()) {
                //gate.set(globals.gate.open);
                if (Objects.equals(robotLocation, "Far Zone")) {
                    intake.set(.6);
                    transfer.set(0.6);
                } else {
                    intake.set(0.8);
                    transfer.set(0.8);
                }
            }
        } else {
            launch1.set(0);
            launch2.set(0);
            launch = false;
        }

        if (!launch && g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(0.7);
            transfer.set(0.2);
            //gate.set(globals.gate.close);
            zoom = true;
        } else if (!launch && !g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(0);
            transfer.set(0);
        }

        follower.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true);
        if (g1.getButton(GamepadKeys.Button.DPAD_UP )) {
            //follower.setStartingPose(new Pose(136, 8, Math.toRadians(90)));
            follower.setPose(new Pose(14.731707317073187, 79.64878048780488, Math.PI/2));
        }

        telemetry.addData("dist", Math.pow(Math.pow(follower.getPose().getX() , 2) + Math.pow(144 - follower.getPose().getY(), 2), 0.5));

        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", globals.testing.targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        launchCalc();
        velocityCalculation();


    }

    private void launchCalc() {
        double dist = Math.pow(Math.pow(follower.getPose().getX() , 2) + Math.pow(144 - follower.getPose().getY(), 2), 0.5);
        double xpos = follower.getPose().getX();
        double ypos = follower.getPose().getY();
        robotZone.setPosition(xpos, ypos);
        robotZone.setRotation(follower.getPose().getHeading());
        if (robotZone.isInside(closeLaunchZone)) {
            targetRPM = 2414.2 * Math.exp(0.0036 * dist);
            hoodAngle = 147.8 * Math.log(dist) - 441.52;
            robotLocation = "Close Zone";
        } else if (robotZone.isInside(farLaunchZone)) {
            targetRPM = 13.09 * dist + 2164.9;
            hoodAngle = 240;
            robotLocation = "Far Zone";
        } else {
            targetRPM = 0;
            hoodAngle = 70;
            robotLocation = "No Zone";
        }

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        double currentheading = wrap360(Math.toDegrees(follower.getPose().getHeading()));
        double targetangle = wrap360(Math.toDegrees(Math.atan2(144.0 - y, 0.0 - x)));
        double relDeg = shortestDiffDeg(targetangle, currentheading);
        double relativeangle = wrap360(180.0 + relDeg);

        if (relativeangle < 5) relativeangle = 180.0;
        if (relativeangle > 355) relativeangle = 180.0;

        turret.set(relativeangle);

    }
    private void velocityCalculation() {

        double goalX = 0.0;
        double goalY = 144.0;

        double dx = goalX - follower.getPose().getX();
        double dy = goalY - follower.getPose().getY();

        double angleToGoal = Math.atan2(dy, dx);

        Vector VelocityVector = follower.getVelocity();

        double netVelocityMagnitude = VelocityVector.getMagnitude();
        double angle = Math.PI - VelocityVector.getTheta();


        double relativeAngle = angle - angleToGoal;


        tangentVelocity = -netVelocityMagnitude * Math.cos(relativeAngle);
        normalVelocity = -netVelocityMagnitude * Math.sin(relativeAngle);

        telemetry.addData("Tangent Vel (m/s)", tangentVelocity);
        telemetry.addData("Normal Vel (m/s)", normalVelocity);
    }
    private double wrap360(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }
    private double shortestDiffDeg(double targetangle, double currentheading) {
        double diff = wrap360(targetangle) - wrap360(currentheading);
        diff = (diff + 540.0) % 360.0 - 180.0;
        return diff;
    }

    private double hoodClamp(double ang) {
        if (ang < 40) {
            ang = 40;
        } else if (ang > 240) {
            ang = 240;
        }
        return ang;
    }



    public void RPM() {
        double currentTime = getRuntime();
        int currentPosition = launch1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }
}
