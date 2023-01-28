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
    public TrajectorySequence rHJSecondCone;
    public TrajectorySequence rHJParkOne;
    public TrajectorySequence rHJParkTwo;
    public TrajectorySequence rHJParkThree;

    public TrajectorySequence leftHighJunction;
    public TrajectorySequence lHJFirstCone;
    public TrajectorySequence lHJSecondCone;
    public TrajectorySequence lHJParkOne;
    public TrajectorySequence lHJParkTwo;
    public TrajectorySequence lHJParkThree;

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
                .splineToLinearHeading(new Pose2d(57, 15.5, Math.toRadians(-94)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }

    public void RHJFirstCone(Pose2d pose)
    {
        rHJFirstCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(51, -17), Math.toRadians(-92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(51, -22, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.3)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(52, 15.4))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(57, 15.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }
    public void RHJSecondCone(Pose2d pose)
    {
        rHJSecondCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(52, -17), Math.toRadians(-92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(52, -22, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.4)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(52, 15.4))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(57, 15.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .waitSeconds(1.0)
                // Move away from pole
                .splineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }

    public void RHJParkOne(Pose2d pose)
    {
        rHJParkOne = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 30, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }
    public void RHJParkTwo(Pose2d pose)
    {
        rHJParkTwo = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 7, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }
    public void RHJParkThree(Pose2d pose)
    {
        rHJParkThree = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, -16, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }

    public void LeftHighJunction(Pose2d pose)
    {
        leftHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move Forward away from the wall
                .splineToConstantHeading(new Vector2d(8, 5), Math.toRadians(0))
                // Move towards the pole
                .splineToConstantHeading(new Vector2d(54, 5), Math.toRadians(0))
                // Move to the pole while turning
                .splineToLinearHeading(new Pose2d(58, -5, Math.toRadians(94)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristRight())
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(53, -5, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }
    public void LHJFirstCone(Pose2d pose)
    {
        lHJFirstCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(53, 29), Math.toRadians(92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(53, 34, Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.3)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(53, -5))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(58, -5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristRight())
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(53, -5, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }
    public void LHJSecondCone(Pose2d pose)
    {
        lHJSecondCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(53, 29), Math.toRadians(92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(53, 34, Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.4)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(53, -5))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(58, -5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristRight())
                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .waitSeconds(1.0)
                // Move away from pole
                .splineToLinearHeading(new Pose2d(53, -5, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }

    public void LHJParkOne(Pose2d pose)
    {
        lHJParkOne = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 28, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }
    public void LHJParkTwo(Pose2d pose)
    {
        lHJParkTwo = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 4, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }
    public void LHJParkThree(Pose2d pose)
    {
        lHJParkThree = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, -18, Math.toRadians(92)), Math.toRadians(0))
                .build();
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
