package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

import java.util.function.Supplier;

@Config
@Autonomous(name = "BasketPP path", group = "Autos")
public class BasketPP extends AutoCommandBasePP {
    // For Basket Scoring
    public static Pose2dHelperClass Basket = new Pose2dHelperClass(-56, -56, 45);
    public static Pose2dHelperClass BasketForSpline = new Pose2dHelperClass(-59, -54.5, 60);
    public static Pose2dHelperClass S1Basket = new Pose2dHelperClass(-60.124, -54, Math.toDegrees(1.1506));
    public static Pose2dHelperClass PreloadBasket = new Pose2dHelperClass(-58, -53, Math.toDegrees(1.1506));
    public static Pose2dHelperClass S2Basket = new Pose2dHelperClass(-64.694, -53, Math.toDegrees(1.621));

    // The right sample
    public static Pose2dHelperClass S1 = new Pose2dHelperClass(-59.7, -48.5, Math.toDegrees(1.12748));
    // The middle sample
    public static Pose2dHelperClass S2 = new Pose2dHelperClass(-63, -50.8168, Math.toDegrees(1.4281));
    // The left sample
    public static Pose2dHelperClass S3 = new Pose2dHelperClass(-61.4, -48.8, 113);

    // Middle point for spline
    public static Pose2dHelperClass splinePoint1 = new Pose2dHelperClass(-24.5, -5, 0);
    public static Pose2dHelperClass splinePoint2 = new Pose2dHelperClass(-24.5, 0, 0);

    public static long basketWaitMs = 630;
    public static long basketWaitForAutoPickMs = 0;
    public static long firstBasketWaitMs = 520;
    public static long pick2Handoff = 0;
    public static long startStowToPath = 200;
    public static long stowedToGrabDelay = 200;
    public static long stopLinearToLLM = 0;
    public static long S3SlideDelay = 0;

    public static double S1TurnPos = 0.35;
    public static double S2TurnPos = 0.25;
    public static double S3TurnPos = 0.17;

    public static Pose startPose = new Pose(-40.13, -63.82, Math.toRadians(90));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public void initializeSuperStructure() {
        follower.setStartingPose(startPose);
//        slide.stow();
//        slide.openIntakeClaw();
//        slide.backwardSlideExtension();
//        liftClaw.closeClaw();
//        liftClaw.foldLiftArm();
    }

    private Pose helperToPose(Pose2dHelperClass helper) {
        return new Pose(helper.getX(), helper.getY(), Math.toRadians(helper.getHeading()));
    }

    @Override
    public Command runAutoCommand() {
//        Supplier<Command> slideExtendCommand = () -> new InstantCommand(() -> slide.forwardSlideExtension());

        return new SequentialCommandGroup(
                new InstantCommand(() -> follower.setStartingPose(startPose)),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(PreloadBasket)))))
                        .alongWith(),
                new WaitCommand(200),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(S1)))))
                        .alongWith(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(S1Basket)))))
                        .alongWith(),
                new WaitCommand(basketWaitMs),
                stowArmFromBasket()
                        .alongWith(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(S2)))))
                        .alongWith(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(S1Basket)))))
                        .alongWith(),
                new WaitCommand(basketWaitMs),
                stowArmFromBasket()
                        .alongWith(),
                                new WaitCommand(S3SlideDelay).andThen(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(S3)))))
                        .alongWith(),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(BasketForSpline)))))
                        .alongWith(),
                new WaitCommand(basketWaitMs),
                stowArmFromBasket()
                        .alongWith(
                                new WaitCommand(startStowToPath)
                                        .andThen(
                                                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(splinePoint1)))))
                                                        .andThen(new WaitCommand(stopLinearToLLM)))),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(BasketForSpline)))))
                        .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
                wait(follower, basketWaitForAutoPickMs),
                stowArmFromBasket()
                        .alongWith(
                                slowHandoff(),
                                new WaitCommand(startStowToPath)
                                        .andThen(
                                                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(splinePoint2)))))
                                                        .andThen(new WaitCommand(stopLinearToLLM)))),
                followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(helperToPose(BasketForSpline)))))
                        .alongWith(new WaitCommand(pick2Handoff).andThen()),
                wait(follower, basketWaitForAutoPickMs));
    }
}
