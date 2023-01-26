package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryBook
{
    SampleMecanumDrive drive;
    ExtraOpModeFunctions extras;

    public TrajectorySequence rightHighJunction;
    public TrajectorySequence rHJFirstCone;
    public TrajectorySequence rHJOne;
    public TrajectorySequence rHJTwo;
    public TrajectorySequence rHJThree;

    public TrajectorySequence leftHighJunction;
    public TrajectorySequence lHJFirstCone;
    public TrajectorySequence lHJOne;
    public TrajectorySequence lHJTwo;
    public TrajectorySequence lHJThree;

    public TrajectorySequence teleOpPoleLeft;
    public TrajectorySequence teleOpConeLeft;

    public TrajectorySequence teleOpPoleRight;
    public TrajectorySequence teleOpConeRight;

    public TrajectorySequence leftParkFromStart;
    public TrajectorySequence middleParkFromStart;
    public TrajectorySequence rightParkFromStart;



    public TrajectoryBook (SampleMecanumDrive drivePass, ExtraOpModeFunctions extrasPass)
    {
        drive = drivePass;
        extras = extrasPass;
    }

    public void RightHighJunction(Pose2d pose)
    {
        rightHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move Forward away from the wall
                .splineToConstantHeading(new Vector2d(8, 5), Math.toRadians(0))
                // Move towards the pole
                .splineToConstantHeading(new Vector2d(54, 5), Math.toRadians(0))
                // Move to the pole while turning
                .splineToLinearHeading(new Pose2d(57, 17, Math.toRadians(-94)), Math.toRadians(0))
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(3.0)
                // Move away from pole
                .splineToLinearHeading(new Pose2d(53, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }

    public void RHJFirstCone(Pose2d pose)
    {
        rHJFirstCone = drive.trajectorySequenceBuilder(pose)
            // Move close to stack
            .splineToConstantHeading(new Vector2d(53, -17), Math.toRadians(-92))
            // Slow move to stack
            .lineToLinearHeading(new Pose2d(53, -22, Math.toRadians(-92)),
                    SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            //.UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
            .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
            // Move close to the pole
            .splineToConstantHeading(new Vector2d(53, -17), Math.toRadians(-92))
            // Move to pole
            .splineToLinearHeading(new Pose2d(57, 17, Math.toRadians(-92)), Math.toRadians(0))
            //.UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
            //.UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
            //.waitSeconds(0.5)
            .build();
    }

    public void RHJOne(Pose2d pose)
    {

    }
    public void RHJTwo(Pose2d pose)
    {

    }
    public void RHJThree(Pose2d pose)
    {

    }

    public void LeftHighJunction(Pose2d pose)
    {
        leftHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move left
                .splineToConstantHeading(new Vector2d( 3, -19), Math.toRadians(0))
                // Move Forward
                .lineToLinearHeading(new Pose2d(50, -19, Math.toRadians(0)))
                // Move Right and turn
                .splineTo(new Vector2d(52, -11), Math.toRadians(87))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                // Move back to stack
                .lineToLinearHeading(new Pose2d(52, 25, Math.toRadians(87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                // Move back to the high junction
                .lineToLinearHeading(new Pose2d(52, -11, Math.toRadians(87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .build();
    }
    public void LHJOne(Pose2d pose)
    {

    }
    public void LHJTwo(Pose2d pose)
    {

    }
    public void LHJThree(Pose2d pose)
    {

    }

    public void TeleOpPoleLeft (Pose2d pose)
    {
        teleOpPoleLeft = drive.trajectorySequenceBuilder(pose)
                // Move to the high pole
                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(180)))
                // Move elevator to the high pole position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                // Move closer to the high pole
                .lineToLinearHeading(new Pose2d(50, -4, Math.toRadians(180)))
                .build();
    }

    public void LeftParkFromStart (Pose2d pose)
    {
        leftParkFromStart = drive.trajectorySequenceBuilder(pose)
                .lineToLinearHeading(new Pose2d(5, 29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(30, 29, Math.toRadians(0)))
                .build();
    }

    public void MiddleParkFromStart (Pose2d pose)
    {
        middleParkFromStart = drive.trajectorySequenceBuilder(pose)
                .lineToLinearHeading(new Pose2d(8, 5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(30, 5, Math.toRadians(0)))
                .build();
    }

    public void RightParkFromStart (Pose2d pose)
    {
        rightParkFromStart = drive.trajectorySequenceBuilder(pose)
                .lineToLinearHeading(new Pose2d(5, -19, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(28, -19, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(30, -19, Math.toRadians(0)))
                .build();
    }

}
