package org.firstinspires.ftc.teamcode.deprs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.globals;


@TeleOp(name = "launch")
public class launcher extends OpMode {
    private Motor launch1, launch2, intake, transfer;
    private int lastPosition;
    private double lastTime;
    private GamepadEx g1;
    private double launchpower;
    private double RPM, secantRPM, previousRPM, rpmDist, oldRPM;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private ElapsedTime timer = new ElapsedTime();
    private double launchRPM;
    private SquIDFController launchSQUIDF = new SquIDFController(globals.launcher.squP, globals.launcher.squI, globals.launcher.squD, globals.launcher.squKv * globals.launcher.targetRPM + globals.launcher.squKs);
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

        intake = new Motor(hardwareMap, "spindexer");
        transfer = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);



        g1 = new GamepadEx(gamepad1);
        launchSQUIDF.setTolerance(20);
        launchPIDF.setTolerance(50);
    }

    @Override
    public void loop() {
        telemetry.update();

        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        launchPIDF.setSetPoint(globals.launcher.targetRPM);
        launchpower = launchPIDF.calculate(RPM);

        if (globals.launcher.launcherOn) {
            launch1.set(launchpower + globals.launcher.kv * globals.launcher.targetRPM + globals.launcher.ks);
            launch2.set(launchpower + globals.launcher.kv * globals.launcher.targetRPM + globals.launcher.ks);
          //if (launchPIDF.atSetPoint()){
            intake.set(1);
            transfer.set(1);
          //}
        } else {
            launch1.set(0);
            launch2.set(0);
        }

        if (g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(1);
            transfer.set(0.2);
        }

        if (!g1.getButton(GamepadKeys.Button.TRIANGLE) && !g1.getButton(GamepadKeys.Button.CROSS)) {
            intake.set(0);
            transfer.set(0);
        }
        RPM();

        if (Math.abs(previousRPM - RPM )> 300) {
            launchRPM = previousRPM;
        }

        telemetry.addData("loop time", timer.seconds());
        timer.reset();
        telemetry.addData("at speed", globals.launcher.SquidOn ? launchSQUIDF.atSetPoint() : launchPIDF.atSetPoint());
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
}
