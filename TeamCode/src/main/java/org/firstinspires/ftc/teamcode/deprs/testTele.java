package org.firstinspires.ftc.teamcode.deprs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
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

@TeleOp (name = "testTele")
public class testTele extends OpMode {

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
        launchPIDF.setTolerance(200);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        turret.set(globals.testing.turretAng);
        hood.set(globals.testing.hoodAng);
        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        follower.update();
        RPM();
        if (g1.getButton(GamepadKeys.Button.CROSS)) {
            launchPIDF.setSetPoint(globals.testing.targetRPM);
            launchpower = launchPIDF.calculate(RPM);
            launch = true;
            if (RPM < 400) {
                launch1.set(0.4);
                launch2.set(0.4);
            } else {

                launch1.set(launchpower + globals.launcher.kv * globals.testing.targetRPM + globals.launcher.ks);
                launch2.set(launchpower + globals.launcher.kv * globals.testing.targetRPM + globals.launcher.ks);
            }


            if (launchPIDF.atSetPoint()) {

                intake.set(.8);
                transfer.set(0.8);
            }
        } else {
            launch1.set(0);
            launch2.set(0);
            launch = false;
        }

        if (!launch && g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(0.7);
            transfer.set(0.2);
            zoom = true;
        } else if (!launch && !g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(0);
            transfer.set(0);
        }

        follower.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true);

        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", globals.testing.targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);

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
