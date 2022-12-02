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
    public TrajectorySequence rHJStacking;
    public TrajectorySequence leftHighJunction;
    public TrajectorySequence lHJStacking;

    public TrajectoryBook (SampleMecanumDrive drivePass, ExtraOpModeFunctions extrasPass)
    {
        drive = drivePass;
        extras = extrasPass;
    }

    public void RightHighJunction(Pose2d pose)
    {
        rightHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move left
                .splineToConstantHeading(new Vector2d(5, 19), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(5, 19, Math.toRadians(0)))
                // Move Forward
                .lineToLinearHeading(new Pose2d(45, 19, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(52, 19, Math.toRadians(0)))
                // Move Right and turn
                .splineTo(new Vector2d(52, 11), Math.toRadians(-87))
                //.lineToLinearHeading(new Pose2d(52, 11, Math.toRadians(-87)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .build();
    }
    public void RHJStacking(Pose2d pose)
    {
        rHJStacking = drive.trajectorySequenceBuilder(pose)
                // Move back to stack
                .lineToLinearHeading(new Pose2d(52, -25, Math.toRadians(-87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                // Move back to the high junction
                .lineToLinearHeading(new Pose2d(52, 11, Math.toRadians(-87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .build();
    }

    public void LeftHighJunction(Pose2d pose)
    {
        leftHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move left
                .splineToConstantHeading(new Vector2d( 3, -19), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(5, 19, Math.toRadians(0)))
                // Move Forward
                .lineToLinearHeading(new Pose2d(45, -19, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(52, 19, Math.toRadians(0)))
                // Move Right and turn
                .splineTo(new Vector2d(52, -11), Math.toRadians(87))
                //.lineToLinearHeading(new Pose2d(52, 11, Math.toRadians(-87)))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .build();
    }
    public void LHJStacking(Pose2d pose)
    {
        lHJStacking = drive.trajectorySequenceBuilder(pose)
                // Move back to stack
                .lineToLinearHeading(new Pose2d(52, 25, Math.toRadians(87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                // Move back to the high junction
                .lineToLinearHeading(new Pose2d(52, -11, Math.toRadians(87)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .build();
    }
}
