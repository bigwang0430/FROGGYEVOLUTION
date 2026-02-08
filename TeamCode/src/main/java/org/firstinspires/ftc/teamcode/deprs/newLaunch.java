package org.firstinspires.ftc.teamcode.deprs;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "newLaunch")
public class newLaunch extends OpMode {

    private Vector robotToGoal;
    private double hoodAngle, turretAng, turretOffset;
    private Motor launch1, launch2, intake, transfer;
    private ServoEx t1, hood;
    private int lastPosition;
    private double lastTime;
    private GamepadEx g1;
    private double launchpower;
    private double RPM, secantRPM, previousRPM, rpmDist, oldRPM;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private ElapsedTime timer = new ElapsedTime();
    private double launchRPM;
    private double flywheelSpeed;
    private Follower follower;
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
        intake = new Motor(hardwareMap, "spindexer");
        transfer = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        t1 = new ServoEx(hardwareMap, "t1", 360, AngleUnit.DEGREES);
        t1.set(180);


        hood = new ServoEx(hardwareMap, "hood", 300, AngleUnit.DEGREES);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(16, 75, Math.PI/2)); //TEMPORARY
    }

    @Override
    public void loop() {
        RPM();

        drive();
        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);


        Pose robot = new Pose (16, 75, Math.PI/2);
        robotToGoal = shooterConstants.blueGoal.getAsVector().minus(robot.getAsVector());
        if (g1.getButton(GamepadKeys.Button.CROSS)) {
            launchPIDF.setSetPoint(exitVeltoRPM(shotVector()[1]));
            launchpower = launchPIDF.calculate(RPM);
            launch1.set(launchpower + globals.launcher.kv * setRPM(flywheelSpeed) + globals.launcher.ks);
            launch2.set(launchpower + globals.launcher.kv * setRPM(flywheelSpeed) + globals.launcher.ks);
//            hood.set(Math.toDegrees(hoodAngletoTicks(shotVector()[0])));
            if (launchPIDF.atSetPoint()) {
                intake.set(1);
                transfer.set(1);
            }
        } else {
            launch1.set(0);
            launch2.set(0);
            intake.set(0);
            transfer.set(0);
        }



        if (Math.abs(previousRPM - RPM )> 300) {
            launchRPM = previousRPM;
        }
        telemetry.addData("s", Math.toDegrees(hoodAngletoTicks(shotVector()[0])));
        telemetry.addData( "b", exitVeltoRPM(shotVector()[1]));

//        telemetry.addData("at speed", launchPIDF.atSetPoint());
//        telemetry.addData("target", globals.launcher.targetRPM);
//        telemetry.addData("rpm", RPM);
//        telemetry.addData("power", launchpower);
//        telemetry.addData("secsnat", secantRPM);
//        telemetry.addData("launch", launchRPM);
//        TelemetryPacket rpmPacket = new TelemetryPacket();
//        rpmPacket.put("RPM", RPM);
//
//        TelemetryPacket powerPacket = new TelemetryPacket();
//        powerPacket.put("targetRPM", globals.launcher.targetRPM);
//
//        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
//        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        follower.update();
        telemetry.update();
    }

    private double exitVeltoRPM(double v) {
        double ms = v * 0.0254;
        return 758.9 * ms - 429.5;
    }
    public void drive() {



        if (g1.getButton(GamepadKeys.Button.DPAD_UP )) {
            //follower.setStartingPose(new Pose(136, 8, Math.toRadians(90)));
            follower.setPose(new Pose(14.731707317073187, 79.64878048780488, Math.PI/2));
        }

        if (g1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            follower.setPose(new Pose(135.55121951219513, 9.404878048780477, Math.PI/2));
        }
        follower.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true);
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

    private double hoodAngletoTicks(double ang) {
        return MathFunctions.clamp((ang - 28)/0.141 , 0, 240);
    }

    private double[] shotVector() {

        double g = 386.09; // inch/s
        double robotHeading = follower.getHeading();
        double x = robotToGoal.getMagnitude() - shooterConstants.passThroughDist;
        double y = shooterConstants.scoreHeight;
        double a = shooterConstants.scoreAngle;

        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), shooterConstants.minHood, shooterConstants.maxHood);
        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector robotVelocity = new Vector (0.00001, 0);

        double coordinateTheta = robotVelocity.getTheta() - robotToGoal.getTheta();
        double tangentVelocity = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double normalVelocity = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double verticalVelocoty = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + tangentVelocity;
        double nvr = Math.sqrt(ivr * ivr + normalVelocity * normalVelocity);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp((Math.atan(verticalVelocoty / nvr)), shooterConstants.minHood, shooterConstants.maxHood);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        turretOffset = Math.atan(normalVelocity / ivr);

        return new double[] {hoodAngle, flywheelSpeed, turretOffset};
    }

    private double setHood(double hood) {
        return hood;
    }
    private double setRPM(double velocity) {
        // Given a specific velocity, provide an RPM
        return velocity;
    }

}
