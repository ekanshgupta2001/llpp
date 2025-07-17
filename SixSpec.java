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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Example Auto Blue", group = "Examples")
public class SixSpec extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int targetPos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime sequenceTimer = new ElapsedTime();
    private ElapsedTime sampleTimer = new ElapsedTime();
    private DcMotor SlideMotor;
    private DcMotor ArmMotor;
    private CRServo intakeMotorL = null;
    private CRServo intakeMotorR = null;
    private CRServo rotateL = null;
    private CRServo rotateR = null;
    private Limelight3A limelight;
    double sampleY = 0;
    double sampleX = 0;
    public static double P = 0.14;
    public static double I = 0.0;
    public static double D = 0.001;

    private int pathState;

    private final Pose startPose = new Pose(67, 109, Math.toRadians(270));
    private final Pose scorefirst = new Pose(67, 111, Math.toRadians(270));
    private final Pose extend = new Pose(sampleX + 67, 111, Math.toRadians(270));
    private final Pose dropandpick = new Pose(80, 112, Math.toRadians(90));
    private final Pose score = new Pose(67, 121, Math.toRadians(270));
    private final Pose pickup1Pose = new Pose(80, 135, Math.toRadians(270));
    private final Pose dropoff1 = new Pose(80, 112, Math.toRadians(270));
    private final Pose pickup2Pose = new Pose(88, 135, Math.toRadians(270));
    private final Pose dropoff2 = new Pose(88, 112, Math.toRadians(270));
    private final Pose pickup3Pose = new Pose(95, 135, Math.toRadians(90));
    private final Pose pickup1 = new Pose(95, 112, Math.toRadians(90));
    private final Pose score1 = new Pose(61, 135, Math.toRadians(270));
    private final Pose pickup2 = new Pose(80, 112, Math.toRadians(90));
    private final Pose score2 = new Pose(57, 135, Math.toRadians(270));
    private final Pose pickup3 = new Pose(80, 112, Math.toRadians(90));
    private final Pose score3 = new Pose(70, 135, Math.toRadians(270));
    private final Pose parkPose = new Pose(67, 109, Math.toRadians(270));


    private Path scorePreload, park;
    private PathChain pickUpSample, dropSub, scoreSub, grabPickup1, dropPickup1, grabPickup2, dropPickup2, grabPickup3, dropPickup3, scoreFirst, pickUpSec, scoreSec, pickupThird, scoreThird;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorefirst)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorefirst.getHeading());

        pickUpSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorefirst), new Point(extend)))
                .setLinearHeadingInterpolation(scorefirst.getHeading(), extend.getHeading())
                .build();

        dropSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(extend), new Point(dropandpick)))
                .setLinearHeadingInterpolation(extend.getHeading(), dropandpick.getHeading())
                .build();

        scoreSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropandpick), new Point(score)))
                .setLinearHeadingInterpolation(dropandpick.getHeading(), score.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(score.getHeading(), pickup1Pose.getHeading())
                .build();

        dropPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(dropoff1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), dropoff1.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoff1), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(dropoff1.getHeading(), pickup2Pose.getHeading())
                .build();

        dropPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(dropoff2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), dropoff2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoff1), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(dropoff1.getHeading(), pickup3Pose.getHeading())
                .build();

        dropPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(pickup1)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup1.getHeading())
                .build();

        scoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1), new Point(score1)))
                .setLinearHeadingInterpolation(pickup1.getHeading(), score1.getHeading())
                .build();

        pickUpSec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1), new Point(pickup2)))
                .setLinearHeadingInterpolation(score1.getHeading(), pickup2.getHeading())
                .build();

        scoreSec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(score2)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), score2.getHeading())
                .build();

        pickupThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2), new Point(pickup3)))
                .setLinearHeadingInterpolation(score2.getHeading(), pickup3.getHeading())
                .build();

        scoreThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3), new Point(score3)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), score3.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(score3), new Point(parkPose)));
        park.setLinearHeadingInterpolation(score3.getHeading(), parkPose.getHeading());


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    ArmMotor.setTargetPosition(1200);
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {
                    LLResult result = limelight.getLatestResult();
                    long staleness = result.getStaleness();
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
                        sampleX = x;
                        sampleY = y;
                        telemetry.addData("Color Target", "takes up " + area + "% of the image");
                    }
                    if(colorResults == null){
                        telemetry.addData("Samples detected: ", null);
                    }
                    int distance = (int) ((sampleY - 24) / tan(-34 - 10));

                    follower.followPath(pickUpSample);
                    SlideMotor.setTargetPosition(distance);
                    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    ArmMotor.setTargetPosition(400);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(pathTimer.getElapsedTime() > 0.5){
                        intakeMotorL.setPower(1.0);
                        intakeMotorR.setPower(-1.0);
                    }
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    ArmMotor.setTargetPosition(700);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(dropSub);
                    if(pathTimer.getElapsedTime() > 0.5){
                        intakeMotorL.setPower(-1.0);
                        intakeMotorR.setPower(1.0);
                    }
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTime() > 0.5){
                        intakeMotorL.setPower(1.0);
                        intakeMotorR.setPower(-1.0);
                    }
                    ArmMotor.setTargetPosition(1100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(scoreSub);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    ArmMotor.setTargetPosition(400);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(grabPickup1);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(dropPickup1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(dropPickup2);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(dropPickup3);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(scoreFirst);
                    ArmMotor.setTargetPosition(1100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(pickUpSec);
                    ArmMotor.setTargetPosition(400);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSec);
                    ArmMotor.setTargetPosition(1100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(pickupThird);
                    ArmMotor.setTargetPosition(400);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(scoreThird);
                    ArmMotor.setTargetPosition(1100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(park);
                    ArmMotor.setTargetPosition(0);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(17);
                }
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

        SlideMotor = hardwareMap.get(DcMotor.class, "Slide Motor");
        SlideMotor.setDirection(DcMotor.Direction.FORWARD);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotateL = hardwareMap.get(CRServo.class, "rotateIntakeLeft");
        rotateR = hardwareMap.get(CRServo.class, "rotateIntakeRight");
        rotateL.setDirection(DcMotorSimple.Direction.FORWARD);
        rotateR.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotorL = hardwareMap.get(CRServo.class, "intakeMotorLeft");
        intakeMotorR = hardwareMap.get(CRServo.class, "intakeMotorRight");
        intakeMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorR.setDirection(DcMotorSimple.Direction.FORWARD);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(75);
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