package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmy.marrow.TimerEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
//import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.time.chrono.IsoEra;
import java.util.List;


//USING BALL TRACKING DOESNT CHANGE END PICKUP POSE. MAY BE AN ISSUE
@Autonomous
public class FROGTONOMOUSTRACKING extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    public String pattern = "ppg";
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
    public TimerEx intaketimer = new TimerEx(30);
    public TimerEx timer = new TimerEx(30);
    public TimerEx launchTimer = new TimerEx(30);
    public int balls = 3;
    public double sdxPower = 0.0;
    public int launchnum = 0;
    public double target = 0.0;
    private enum sdxCase {
        spinUp,
        launch3Slow,
        launch3Fast,
        resetSpindexer
    } sdxCase sdxState = sdxCase.spinUp;

    public boolean hunted = false;


    //PATHS

    //FUNCTIONS

    //SUBSYSTEMS
    public class visionsubsys extends SubsystemBase {
        private Limelight3A limelight;
        private double numspeed = 0.4;
        private Motor fl, bl, fr, br;

        public visionsubsys(HardwareMap map) {
            limelight = map.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();

            FtcDashboard.getInstance().startCameraStream(limelight, 30);

            limelight.pipelineSwitch(2);

            fl = new Motor(map, "fl");
            fl.setInverted(true);
            fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            fl.setRunMode(Motor.RunMode.RawPower);

            bl = new Motor(map, "bl");
            bl.setInverted(true);
            bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            bl.setRunMode(Motor.RunMode.RawPower);

            fr = new Motor(map, "fr");
            fr.setInverted(false);
            fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            fr.setRunMode(Motor.RunMode.RawPower);

            br = new Motor(map, "br");
            br.setInverted(false);
            br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            br.setRunMode(Motor.RunMode.RawPower);

        }


        public void balltracking() {
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid() || result.getStaleness() >= 300) {
                strafe(0);
                return;
            }

            List<LLResultTypes.ColorResult> balls = result.getColorResults();
            if (balls == null || balls.isEmpty()) {
                strafe(0);
                return;
            }

            LLResultTypes.ColorResult ball = balls.get(0);

            double txPx = ball.getTargetXPixels(); // pixels from crosshair (can be +/-)
            if (txPx > 170) {
                strafe(+1);
            } else if (txPx < 150) {
                strafe(-1);
            } else {
                strafe(0);
                hunted = true;
            }

            telemetry.addData("hunted", hunted);
            telemetry.addData("tx", txPx);
            telemetry.update();
        }

        public void strafe(int num) {
            fl.set(num*numspeed);
            fr.set(-num*numspeed);
            bl.set(-num*numspeed);
            br.set(num*numspeed);
        }

        public boolean done() {
            if (hunted){
                return  true;
            }
            return false;
        }
    }

    //COMMANDS
    public static class froggyhunting extends CommandBase {
        private final visionsubsys visionsubsystem;

        public froggyhunting(visionsubsys visionsubsystem){
            this.visionsubsystem = visionsubsystem;
            addRequirements(visionsubsystem);
        }


        @Override
        public void execute() {
            visionsubsystem.balltracking();
        }

        @Override
        public boolean isFinished(){
            return visionsubsystem.done();
        }
    }



    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        visionsubsys visionsubsystem = new visionsubsys(hardwareMap);

        timer.start();

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        sleep(1000);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(180)));//todo

        SequentialCommandGroup froggyroute = new SequentialCommandGroup(
            new froggyhunting(visionsubsystem)
        );

        schedule(froggyroute);

    }


    @Override
    public void run() {
        super.run();
        follower.update();
    }
}




