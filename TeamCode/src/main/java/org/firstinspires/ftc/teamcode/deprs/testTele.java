package org.firstinspires.ftc.teamcode.deprs;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private double RPM, previousRPM, dist, turretAng;
    private ServoEx hood, turret, gate;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private boolean launch, zoom;
    private double targetRPM, hoodAngle, tangentVelocity, normalVelocity;
    private String robotLocation;
    private ElapsedTime timer = new ElapsedTime();
    private String turretState = "Valid";
    //valid, aligned, invalid, notAligned

    private enum launchMode {
        SOTM,
        normal
    } private launchMode currentLaunchMode = launchMode.normal;
    @Override
    public void init() {
        timer.startTime();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        turret = new ServoEx(hardwareMap, "t2", 360, AngleUnit.DEGREES);
        turret.setInverted(true);
        launch1 = new Motor(hardwareMap, "l1", 28, 6000);
        launch2 = new Motor(hardwareMap, "l2", 28, 6000);
        launch1.setRunMode(Motor.RunMode.RawPower);
        launch2.setRunMode(Motor.RunMode.RawPower);
        launch2.setInverted(true);
        launch1.setInverted(false);
        launch1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launch2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        gate = new ServoEx(hardwareMap, "gate");
        gate.set(globals.gate.close);
        intake = new Motor(hardwareMap, "intake");
        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(false);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300, AngleUnit.DEGREES);

        g1 = new GamepadEx(gamepad1);
        launchPIDF.setTolerance(50);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(16, 75, Math.PI/2)); //TEMPORARY
        while (timer.seconds() < 0.5) {
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        follower.setMaxPower(0.8);
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


            if (launchPIDF.atSetPoint() && !robotLocation.equals("No Zone")) {
                gate.set(globals.gate.open);
                if (Objects.equals(robotLocation, "Far Zone")) {
                    intake.set(.65);
                    transfer.set(0.65);
                } else {
                    intake.set(0.85);
                    transfer.set(0.85);
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
            gate.set(globals.gate.close);
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
        telemetry.addData("loop time", timer.seconds());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        timer.reset();

    }

    private void launchCalc() {

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Pose robot = new Pose(x, y);
        robotZone.setPosition(x, y);
        robotZone.setRotation(follower.getPose().getHeading());
        if (follower.getVelocity().getMagnitude() < 5 || robotLocation.equals("Far Zone")) {
            currentLaunchMode = launchMode.normal;
        } else {
            currentLaunchMode = launchMode.SOTM;
        }






        switch (currentLaunchMode) {
            case SOTM:
                Pose stationaryGoal = new Pose(4, 138);
                dist = stationaryGoal.minus(robot).getAsVector().getMagnitude();

                double accelMag = Math.floor(follower.getAcceleration().getMagnitude());
                double accelAngle = Math.toRadians(Math.floor(Math.toDegrees(follower.getAcceleration().getTheta())));
                Vector accel = new Vector(accelMag, accelAngle);
                Vector velocity = follower.getVelocity().plus(new Vector(accel.getMagnitude() * globals.launcher.velTime, accel.getTheta()));
                double distanceDiff = velocity.getMagnitude() * (0.0025 * dist + 0.3871);
                Vector robotVelocity = new Vector (distanceDiff, velocity.getTheta());

                Pose newGoal = new Pose(-robotVelocity.getXComponent() + stationaryGoal.getX(), -robotVelocity.getYComponent() + stationaryGoal.getY());

                double newGoalAngle = Math.atan2(newGoal.getY() - y, newGoal.getX()-x);
                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - newGoalAngle));
                dist = newGoal.minus(robot).getAsVector().getMagnitude();
                telemetry.addData("turretAng", turretAng);
                telemetry.addData("ang", newGoalAngle);
                break;
            case normal:
                Pose goal = new Pose(4, 138);
                Pose target = goal.minus(robot);
                Vector robotToGoal = target.getAsVector();
                double goalAngle = Math.atan2(goal.getY() - y, goal.getX()-x);
                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
                dist = robotToGoal.getMagnitude();
                telemetry.addData("turretAng", turretAng);
                telemetry.addData("roo", follower.getHeading());
                telemetry.addData("ang", goalAngle);
                telemetry.addData("calc dist", dist);
                break;

        }
        if (robotZone.isInside(closeLaunchZone)) {
            targetRPM = 2414.2 * Math.exp(0.0036 * dist);
            if (dist < 24) {
                hoodAngle = 40;
            } else {
                hoodAngle = 147.8 * Math.log(dist) - 441.52;
            }
            robotLocation = "Close Zone";
        } else if (robotZone.isInside(farLaunchZone)) {
            targetRPM = 13.09 * dist + 2164.9;
            hoodAngle = 240;
            robotLocation = "Far Zone";
        } else {
            robotLocation = "No Zone";
        }



        if (Math.abs(turretAng) > 130) {
            turretAng = 0;
        }

        turret.set(clampTurret(turretAng));
    }
    private double wrap(double angle) {
        if (angle < 0) {
            angle =  2*Math.PI + angle;
        }
        return angle;
    }

    private double clampTurret(double ang) {

            ang = 180 - ((ang * 3) / 2) - globals.launcher.turretOffset;
            return ang;
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
