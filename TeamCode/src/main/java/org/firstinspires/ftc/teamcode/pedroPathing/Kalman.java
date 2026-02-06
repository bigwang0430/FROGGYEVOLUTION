package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.globals;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class Kalman extends CommandOpMode {
    private Follower follower;
    private Limelight3A limelight;

    private double xEst = 0, yEst = 0, hEst = 0;   // odometry estimate
    private double pX = globals.kalman.pX0, pY = globals.kalman.pY0, pH = 1; // odometry error

    private static final double M_TO_IN = 39.37007874015748;

    private Pose fusedPose = new Pose(0, 0, 0);
    private Pose odoPose = new Pose(0, 0, 0);

    @Override
    public void initialize() {

        telemetry.update();
        follower = Constants.createFollower(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        sleep(500);

        follower.setStartingPose(new Pose(72, 72, 3*Math.PI/4)); // set whatever start you want

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(0);

        // Initialize estimate to odometry so don't start at 0,0,0
        Pose p = follower.getPose();
        if (p != null) {
            xEst = p.getX();
            yEst = p.getY();
            hEst = p.getHeading();
            fusedPose = new Pose(xEst, yEst, hEst);
            odoPose = p;
        }
    }

    public void updateKalman() {
        Pose odo = follower.getPose();
        if (odo == null) return;

        odoPose = odo;

        // Prediction
        double xPred = odo.getX(), yPred = odo.getY(), hPred = odo.getHeading();
        double pXPred = pX + globals.kalman.qX;
        double pYPred = pY + globals.kalman.qY;
        double pHPred = pH;


        // no measurement = keep position
        xEst = xPred; yEst = yPred; hEst = hPred;
        pX = pXPred; pY = pYPred; pH = pHPred;

        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            Pose3D bot = r.getBotpose();
            if (bot != null) {
                double zX = 72 + (bot.getPosition().y * M_TO_IN);
                double zY = 72 - (bot.getPosition().x * M_TO_IN);


                double kX = pXPred / (pXPred + globals.kalman.rX);
                double kY = pYPred / (pYPred + globals.kalman.rY);

                xEst = xPred + kX * (zX - xPred);
                yEst = yPred + kY * (zY - yPred);

                pX = (1.0 - kX) * pXPred;
                pY = (1.0 - kY) * pYPred;
            }
        }

        fusedPose = new Pose(xEst, yEst, hEst);
    }
    public void setStartPose(Pose setStart) {
        if (setStart == null) return;
        xEst = setStart.getX();
        yEst = setStart.getY();
        hEst = setStart.getHeading();
        fusedPose = new Pose(xEst, yEst, hEst);

        // reset covariances from Globals
        pX = globals.kalman.pX0;
        pY = globals.kalman.pY0;
        pH = 1;
    }

    public boolean isNAN() { return Double.isNaN(xEst) || Double.isNaN(yEst) || Double.isNaN(hEst); }

    @Override
    public void run() {
        super.run();
        follower.update();
        updateKalman();
        telemetry.addData("ODO X", odoPose.getX());
        telemetry.addData("ODO Y", odoPose.getY());

        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            Pose3D bot = r.getBotpose();
            if (bot != null) {
                telemetry.addData("LL X", 72 + (bot.getPosition().y * M_TO_IN)); // keep LL x convention
                telemetry.addData("LL Y", 72 - (bot.getPosition().x * M_TO_IN)); // flipped direction

            }
        }


        telemetry.addData("KAL X", fusedPose.getX());
        telemetry.addData("KAL Y", fusedPose.getY());

        telemetry.addData("pX", pX);
        telemetry.addData("pY", pY);
        telemetry.addData("NAN", isNAN());
        telemetry.update();
    }
}