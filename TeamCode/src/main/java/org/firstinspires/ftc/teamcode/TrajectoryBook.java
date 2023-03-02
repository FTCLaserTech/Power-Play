package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class TrajectoryBook
{
    SampleMecanumDrive drive;
    ExtraOpModeFunctions extras;

    public TrajectorySequence rightHighJunctionFast;
    public TrajectorySequence rHJFirstConeFast;


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

    public TrajectorySequence rightMediumJunction;

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
    public void RightHighJunctionFast(Pose2d pose)
    {
        rightHighJunctionFast = drive.trajectorySequenceBuilder(pose)
                // Move Forward away from the wall
                .splineToConstantHeading(new Vector2d(8, 5), Math.toRadians(0))
                // Move towards the pole
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.elevatorHigh())
                .splineToConstantHeading(new Vector2d(54, 5), Math.toRadians(0))
                // Move to the pole while turning
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.wristLeft())
                .splineToLinearHeading(new Pose2d(58, 16, Math.toRadians(-94)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .build();
    }
    public void RHJFirstConeFast(Pose2d pose)
    {
        rHJFirstConeFast = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.elevatorFive())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-92)), Math.toRadians(0))
                // Move close to stack
                .splineToConstantHeading(new Vector2d(51, -17), Math.toRadians(-92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(52, -22, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> extras.elevatorHigh())
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(52, 17))
                // Move left to pole
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .lineToConstantHeading(new Vector2d(57, 17))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                // Move away from pole
                .splineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }





    public void RightHighJunction(Pose2d pose)
    {
        rightHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move Forward away from the wall
                .splineToConstantHeading(new Vector2d(8, 5), Math.toRadians(0))
                // Move towards the pole
                .splineToConstantHeading(new Vector2d(54, 5), Math.toRadians(0))
                // Move to the pole while turning
                .splineToLinearHeading(new Pose2d(58, 16, Math.toRadians(-94)), Math.toRadians(0))
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
                .splineToLinearHeading(new Pose2d(52.5, 17, Math.toRadians(-92)), Math.toRadians(0))
                .build();
    }
    public void RHJFirstCone(Pose2d pose)
    {
        rHJFirstCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(52.5, -17), Math.toRadians(-92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(52.5, -22, Math.toRadians(-92)),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.3)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(52, 17))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(57, 17))
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
                .lineToConstantHeading(new Vector2d(52, 17))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(57, 17))
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
                .splineToLinearHeading(new Pose2d(53, 30, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(52, 31, Math.toRadians(0)))
                .build();
    }
    public void RHJParkTwo(Pose2d pose)
    {
        rHJParkTwo = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 7, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(52, 8, Math.toRadians(0)))
                .build();
    }
    public void RHJParkThree(Pose2d pose)
    {
        rHJParkThree = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, -13, Math.toRadians(-92)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(52, -13.1, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, -16, Math.toRadians(0)))
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
                .splineToLinearHeading(new Pose2d(57.5, -6.5, Math.toRadians(94)), Math.toRadians(0))
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
                .splineToLinearHeading(new Pose2d(53, -6, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }
    public void LHJFirstCone(Pose2d pose)
    {
        lHJFirstCone = drive.trajectorySequenceBuilder(pose)
                // Move close to stack
                .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(51, 34, Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.3)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(53, -5))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(56, -5))
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
                .splineToConstantHeading(new Vector2d(50, 29), Math.toRadians(92))
                // Slow move to stack
                .lineToLinearHeading(new Pose2d(50, 34, Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                .waitSeconds(0.4)
                // Move backward to the pole
                .lineToConstantHeading(new Vector2d(53, -5))
                // Move left to pole
                .lineToConstantHeading(new Vector2d(55.5, -5))
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
                .splineToLinearHeading(new Pose2d(51, -5, Math.toRadians(92)), Math.toRadians(0))
                .build();
    }

    public void LHJParkOne(Pose2d pose)
    {
        lHJParkOne = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(53, 25, Math.toRadians(92)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(52, 27, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(52, 29, Math.toRadians(0)))
                .build();
    }
    public void LHJParkTwo(Pose2d pose)
    {
        lHJParkTwo = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(51, 7, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(52, 5, Math.toRadians(0)))
                .build();
    }
    public void LHJParkThree(Pose2d pose)
    {
        lHJParkThree = drive.trajectorySequenceBuilder(pose)
                .splineToLinearHeading(new Pose2d(51, -18, Math.toRadians(0)), Math.toRadians(0))
                //.lineToLinearHeading(new Pose2d(52, -17, Math.toRadians(0)))
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

    public void RightMediumJunction(Pose2d pose) {
        rightMediumJunction = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                //.lineTo(new Vector2d(17,9))
                //.splineToLinearHeading(new Pose2d(15,11,Math.toRadians(0)),Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(26,13,Math.toRadians(45)),Math.toRadians(90))

                //.splineTo(new Vector2d(23, 13),Math.toRadians(35))
                //.splineTo(new Vector2d(28, 13),Math.toRadians(35))
                //.splineTo(new Vector2d(33, 11),Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(10, 28,0),Math.toRadians(0))
                //.splineTo(new Vector2d(10, 28),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorMiddle())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristRight())
                .splineTo(new Vector2d(43, 23),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(2)
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> extras.elevatorMiddle())
                //.linetoLinearHeading(new Pose2d(x,y), Math.toRadians(

                .build();
    }
}
