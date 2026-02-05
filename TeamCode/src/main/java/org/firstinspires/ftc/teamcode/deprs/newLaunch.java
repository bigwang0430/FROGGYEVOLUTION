package org.firstinspires.ftc.teamcode.deprs;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.globals;


@TeleOp(name = "newLaunch")
public class newLaunch extends OpMode {
    private Vector robotToGoal;
    private double hoodAngle, turretAng, turretOffset;
    private Motor launch1, launch2;
    private int lastPosition;
    private double lastTime;
    private GamepadEx g1;
    private double launchpower;
    private double RPM, secantRPM, previousRPM, rpmDist, oldRPM;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private ElapsedTime timer = new ElapsedTime();
    private double launchRPM;
    private double flywheelSpeed;
    @Override
    public void init() {
        launch1 = new Motor(hardwareMap, "launch1", 28, 6000);
        launch2 = new Motor(hardwareMap, "launch2", 28, 6000);
        launch1.setRunMode(Motor.RunMode.RawPower);
        launch2.setRunMode(Motor.RunMode.RawPower);
        launch2.setInverted(false);
        launch1.setInverted(true);
        launch1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launch2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        g1 = new GamepadEx(gamepad1);

        launchPIDF.setTolerance(20);
    }

    @Override
    public void loop() {
        RPM();
        shotVector();
        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        launchPIDF.setSetPoint(setRPM(flywheelSpeed));
        launchpower = launchPIDF.calculate(RPM);

        robotToGoal = shooterConstants.blueGoal.getAsVector().minus(follower.getPose().getAsVector());
        if (globals.launcher.launcherOn) {
            launch1.set(launchpower + globals.launcher.kv * setRPM(flywheelSpeed) + globals.launcher.ks);
            launch2.set(launchpower + globals.launcher.kv * setRPM(flywheelSpeed) + globals.launcher.ks);
        } else {
            launch1.set(0);
            launch2.set(0);
        }

        if (Math.abs(previousRPM - RPM )> 300) {
            launchRPM = previousRPM;
        }
        telemetry.addData("at speed", launchPIDF.atSetPoint());
        telemetry.addData("target", globals.launcher.targetRPM);
        telemetry.addData("rpm", RPM);
        telemetry.addData("power", launchpower);
        telemetry.addData("secsnat", secantRPM);
        telemetry.addData("launch", launchRPM);
        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", globals.launcher.targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        telemetry.update();
    }
    public void RPM() {
        if (rpmDist >= 10) {
            oldRPM = RPM;
            rpmDist = 0;
        }
        rpmDist++;
        double currentTime = getRuntime();
        int currentPosition = launch1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;
        secantRPM = (Math.abs(oldRPM - RPM));
        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }

    private void shotVector() {
        double g = 386.09; // inch/s
        double robotHeading = follower.getHeading();
        double x = robotToGoal.getMagnitude() - shooterConstants.passThroughDist;
        double y = shooterConstants.scoreHeight;
        double a = shooterConstants.scoreAngle;

        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), shooterConstants.maxHood, shooterConstants.minHood);
        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector robotVelocity = follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoal.getTheta();
        double tangentVelocity = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double normalVelocity = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double verticalVelocoty = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time * tangentVelocity;
        double nvr = Math.sqrt(ivr * ivr + normalVelocity * normalVelocity);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(Math.atan(verticalVelocoty / nvr), shooterConstants.maxHood, shooterConstants.minHood);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        turretOffset = Math.atan(normalVelocity / ivr);
        turretAng = Math.toDegrees(robotHeading - robotToGoal.getTheta() + turretOffset);
    }

    private double setHood(double hood) {
        return hood;
    }
    private double setRPM(double velocity) {
        // Given a specific velocity, provide an RPM
        return velocity;
    }

}
