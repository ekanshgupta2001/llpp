package pedroPathing.auto;

import static java.lang.Math.tan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Example Auto Blue", group = "Examples")
public class ppllcombo extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int targetPos = 0;
    private DcMotor SlideLeft;
    private DcMotor SlideRight;
    private Limelight3A limelight;

    public static double P = 0.14;
    public static double I = 0.0;
    public static double D = 0.001;

    private int pathState;

    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    private final Pose extend = new Pose(9, 111, Math.toRadians(270));


    private Path move;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        move = new Path(new BezierLine(new Point(startPose), new Point(extend)));
        move.setLinearHeadingInterpolation(startPose.getHeading(), extend.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                double sampleY = 0;
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    LLResultTypes.ColorResult detection = null;
                    double x = detection.getTargetXDegrees();
                    double y = detection.getTargetYDegrees();
                    double area = colorTarget.getTargetArea();
                    sampleY = y;
                    telemetry.addData("Color Target", "takes up " + area + "% of the image");
                }
                int distance = (int) ((sampleY - 24) / tan(-34 - 10));
                SlideLeft.setTargetPosition(distance);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setTargetPosition(distance);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                follower.followPath(move);
                setPathState(1);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
// These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

// Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        SlideLeft = hardwareMap.get(DcMotor.class, "depositSlideLeft");
        SlideRight = hardwareMap.get(DcMotor.class, "depositSlideRight");

        SlideLeft.setDirection(DcMotor.Direction.FORWARD);
        SlideRight.setDirection(DcMotor.Direction.FORWARD);

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public static void pipelineSwitch(int i) {
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

    }


    @Override
    public void stop() {

    }
}

