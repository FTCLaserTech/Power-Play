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

    public TrajectorySequence redCarouselSpinDuck;
    public TrajectorySequence blueCarouselSpinDuck;

    public TrajectorySequence redWarehouseRightBlockDrop;
    public TrajectorySequence redWarehouseMiddleBlockDrop;
    public TrajectorySequence redWarehouseLeftBlockDrop;
    public TrajectorySequence blueWarehouseRightBlockDrop;
    public TrajectorySequence blueWarehouseMiddleBlockDrop;
    public TrajectorySequence blueWarehouseLeftBlockDrop;


    public TrajectorySequence redRightBlockCarousel;
    public TrajectorySequence redMiddleBlockCarousel;
    public TrajectorySequence redLeftBlockCarousel;
    public TrajectorySequence blueRightBlockCarousel;
    public TrajectorySequence blueMiddleBlockCarousel;
    public TrajectorySequence blueLeftBlockCarousel;


    public TrajectorySequence redParkInWarehouseFromCarousel;
    public TrajectorySequence redParkInStorageFromCarousel;
    public TrajectorySequence blueParkInWarehouseCarousel;
    public TrajectorySequence blueParkInStorageFromCarousel;

    public TrajectorySequence redParkInWarehouseFromWarehouse;
    public TrajectorySequence blueParkInWarehouseFromWarehouse;

    public TrajectorySequence redRightDuckBlock;

    public Trajectory blueWarehouseShift;
    public Trajectory redWarehouseShift;

    // These trajectories deliver the duck after it is picked up
    public TrajectorySequence redDuckStorageUnitRight;
    public TrajectorySequence redDuckStorageUnitMiddle;
    public TrajectorySequence redDuckStorageUnitLeft;

    public TrajectorySequence redDuckPark;
    public TrajectorySequence blueDuckStorageUnitRight;
    public TrajectorySequence blueDuckStorageUnitMiddle;
    public TrajectorySequence blueDuckStorageUnitLeft;
    public TrajectorySequence blueDuckPark;
    public TrajectorySequence redCapstoneWarehouse;
    public TrajectorySequence redCapstoneWarehouseShift;
    public TrajectorySequence blueCapstoneWarehouse;
    public TrajectorySequence blueCapstoneWarehouseShift;

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

}
