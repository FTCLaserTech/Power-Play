package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryBook
{
    SampleMecanumDrive drive;
    ExtraOpModeFunctions extras;

    // Right High Junction 1+2
    public TrajectorySequence rightHighJunctionOne;
    public TrajectorySequence rHJFirstCone;
    public TrajectorySequence rHJSecondCone;
    public TrajectorySequence rHJParkOne;
    public TrajectorySequence rHJParkTwo;
    public TrajectorySequence rHJParkThree;

    // Left High Junction 1+2
    public TrajectorySequence leftHighJunctionOne;
    public TrajectorySequence lHJFirstCone;
    public TrajectorySequence lHJSecondCone;
    public TrajectorySequence lHJParkOne;
    public TrajectorySequence lHJParkTwo;
    public TrajectorySequence lHJParkThree;

    // Just Park
    public TrajectorySequence leftParkFromStart;
    public TrajectorySequence middleParkFromStart;
    public TrajectorySequence rightParkFromStart;

    // Right Medium Junction 1+5
    public TrajectorySequence rightMediumJunction;
    public TrajectorySequence rMJStackToJunction;
    public TrajectorySequence rMJJunctionToStack4;
    public TrajectorySequence rMJJunctionToStack3;
    public TrajectorySequence rMJJunctionToStack2;
    public TrajectorySequence rMJJunctionToStack1;
    public TrajectorySequence rMJLeftPark;
    public TrajectorySequence rMJMiddlePark;
    public TrajectorySequence rMJRightPark;

    // Left Medium Junction 1+5
    public TrajectorySequence leftMediumJunction;
    public TrajectorySequence lMJStackToJunction;
    public TrajectorySequence lMJJunctionToStack4;
    public TrajectorySequence lMJJunctionToStack3;
    public TrajectorySequence lMJJunctionToStack2;
    public TrajectorySequence lMJJunctionToStack1;
    public TrajectorySequence lMJLeftPark;
    public TrajectorySequence lMJMiddlePark;
    public TrajectorySequence lMJRightPark;

    // Right Medium Junction 1+5
    public TrajectorySequence rightHighJunction;
    public TrajectorySequence rHJStackToJunction;
    public TrajectorySequence rHJJunctionToStack4;
    public TrajectorySequence rHJJunctionToStack3;
    public TrajectorySequence rHJJunctionToStack2;
    public TrajectorySequence rHJJunctionToStack1;
    public TrajectorySequence rHJLeftPark;
    public TrajectorySequence rHJMiddlePark;
    public TrajectorySequence rHJRightPark;

    // Left Medium Junction 1+5
    public TrajectorySequence leftHighJunction;
    public TrajectorySequence lHJStackToJunction;
    public TrajectorySequence lHJJunctionToStack4;
    public TrajectorySequence lHJJunctionToStack3;
    public TrajectorySequence lHJJunctionToStack2;
    public TrajectorySequence lHJJunctionToStack1;
    public TrajectorySequence lHJLeftPark;
    public TrajectorySequence lHJMiddlePark;
    public TrajectorySequence lHJRightPark;


    public TrajectoryBook (SampleMecanumDrive drivePass, ExtraOpModeFunctions extrasPass)
    {
        drive = drivePass;
        extras = extrasPass;
    }

    public void RightHighJunctionOne(Pose2d pose)
    {
        rightHighJunction = drive.trajectorySequenceBuilder(pose)
                // Move Forward away from the wall
                .splineToConstantHeading(new Vector2d(8, 5), Math.toRadians(0))
                // Move towards the pole
                .splineToConstantHeading(new Vector2d(54, 5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                // Move to the pole while turning
                .splineToLinearHeading(new Pose2d(58, 16, Math.toRadians(-94)), Math.toRadians(0))
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .waitSeconds(0.2)
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

    public void LeftHighJunctionOne(Pose2d pose)
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


// HIGH JUNCTIONS - Right


    public void RightMedJuncInitial(Pose2d pose) {
        rightMediumJunction = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                //Initial Cone
                .splineToConstantHeading(new Vector2d(4, 30),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorMiddle())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristRight())
                .splineToConstantHeading(new Vector2d(33, 29),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, 20),Math.toRadians(0)) // y was 21.5
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                // Turn to face stack
                .splineTo(new Vector2d(50,17),Math.toRadians(-92))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                .lineTo(new Vector2d(50, -26))
                .build();
    }

    public void RMJStackToJunction(Pose2d pose)
    {
            rMJStackToJunction = drive.trajectorySequenceBuilder(pose)

            .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
            .waitSeconds(0.2)
            .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorMiddle())
            .UNSTABLE_addTemporalMarkerOffset(1, () -> extras.wristRight())
            // Move backward to the pole
            .lineTo(new Vector2d(-6, 38))
            .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
            .waitSeconds(0.3)
            .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
            .build();
    }

    public void RMJJunctionToStack4(Pose2d pose)
    {
        rMJJunctionToStack4 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                .lineTo(new Vector2d(1, -3))
                .build();
    }

    public void RMJJunctionToStack3(Pose2d pose)
    {
        rMJJunctionToStack3 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorThree())
                .lineTo(new Vector2d(1, -3))
                .build();
    }

    public void RMJJunctionToStack2(Pose2d pose)
    {
        rMJJunctionToStack2 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorTwo())
                .lineTo(new Vector2d(1, -3))
                .build();
    }

    public void RMJJunctionToStack1(Pose2d pose)
    {
        rMJJunctionToStack1 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(1, -3))
                .build();
    }

    public void RMJLeftPark(Pose2d pose)
    {
        rMJLeftPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineToLinearHeading(new Pose2d(-5, 47, Math.toRadians(2)))
                .build();
    }
    public void RMJMiddlePark(Pose2d pose)
    {
        rMJMiddlePark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, 22))
                .lineToLinearHeading(new Pose2d(-5, 22, Math.toRadians(2)))
                .build();
    }
    public void RMJRightPark(Pose2d pose)
    {
        rMJRightPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, 2))
                .lineToLinearHeading(new Pose2d(-5, 2, Math.toRadians(2)))
                .build();
    }


// HIGH JUNCTIONS - Left


    public void LeftMedJuncInitial(Pose2d pose) {
        leftMediumJunction = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                //Initial Cone
                .splineToConstantHeading(new Vector2d(6, -17),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorMiddle())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .splineToConstantHeading(new Vector2d(35, -14),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(39.5, -9),Math.toRadians(0)) // y was 21.5
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                // Turn to face stack
                .splineTo(new Vector2d(52,0),Math.toRadians(92))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                .lineTo(new Vector2d(52, 39.5))
                .build();
    }

    public void LMJStackToJunction(Pose2d pose)
    {
        lMJStackToJunction = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorMiddle())
                .UNSTABLE_addTemporalMarkerOffset(1, () -> extras.wristLeft())
                // Move backward to the pole
                .lineTo(new Vector2d(-7, -37))
                //.waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                // Move away from pole
                .build();
    }

    public void LMJJunctionToStack4(Pose2d pose)
    {
        lMJJunctionToStack4 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LMJJunctionToStack3(Pose2d pose)
    {
        lMJJunctionToStack3 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorThree())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LMJJunctionToStack2(Pose2d pose)
    {
        lMJJunctionToStack2 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorTwo())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LMJJunctionToStack1(Pose2d pose)
    {
        lMJJunctionToStack1 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LMJLeftPark(Pose2d pose)
    {
        lMJLeftPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, -2))
                .lineToLinearHeading(new Pose2d(-5, -2, Math.toRadians(-2)))
                .build();
    }
    public void LMJMiddlePark(Pose2d pose)
    {
        lMJMiddlePark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, -22))
                .lineToLinearHeading(new Pose2d(-5, -22, Math.toRadians(-2)))
                .build();
    }
    public void LMJRightPark(Pose2d pose)
    {
        lMJRightPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineToLinearHeading(new Pose2d(-5, -47, Math.toRadians(-2)))
                .build();
    }


// HIGH JUNCTIONS - Right


    public void RightHighJuncInitial(Pose2d pose) {
        rightHighJunction = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                //Initial Cone
                .splineToConstantHeading(new Vector2d(4, 30),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .splineToConstantHeading(new Vector2d(44, 29),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .splineTo(new Vector2d(51,24),Math.toRadians(-90))
                .lineTo(new Vector2d(56.5,13))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                // Turn to face stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                .splineToLinearHeading(new Pose2d(52.5, 3,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(52.5, -28))
                .build();
    }

    public void RHJStackToJunction(Pose2d pose)
    {
        rHJStackToJunction = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .UNSTABLE_addTemporalMarkerOffset(1, () -> extras.wristLeft())
                // Move backward to the pole
                .lineTo(new Vector2d(0, 27))
                .splineToLinearHeading(new Pose2d(8, 32,Math.toRadians(-90)),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .build();
    }

    public void RHJJunctionToStack4(Pose2d pose)
    {
        rHJJunctionToStack4 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                .splineToLinearHeading(new Pose2d(3, 23,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(3, -7))
                .build();
    }

    public void RHJJunctionToStack3(Pose2d pose)
    {
        rHJJunctionToStack3 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorThree())
                .splineToLinearHeading(new Pose2d(3, 23,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(3, -7))
                .build();
    }

    public void RHJJunctionToStack2(Pose2d pose)
    {
        rHJJunctionToStack2 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorTwo())
                .splineToLinearHeading(new Pose2d(3, 23,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(3, -7))
                .build();
    }

    public void RHJJunctionToStack1(Pose2d pose)
    {
        rHJJunctionToStack1 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .splineToLinearHeading(new Pose2d(3, 23,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(3, -7))
                .build();
    }

    public void RHJLeftPark(Pose2d pose)
    {
        rHJLeftPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineToLinearHeading(new Pose2d(-5, 47, Math.toRadians(2)))
                .build();
    }
    public void RHJMiddlePark(Pose2d pose)
    {
        rHJMiddlePark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, 22))
                .lineToLinearHeading(new Pose2d(-5, 22, Math.toRadians(2)))
                .build();
    }
    public void RHJRightPark(Pose2d pose)
    {
        rHJRightPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, 2))
                .lineToLinearHeading(new Pose2d(-5, 2, Math.toRadians(2)))
                .build();
    }


// HIGH JUNCTIONS - Left


    public void LeftHighJuncInitial(Pose2d pose) {
        leftHighJunction = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorLow())
                //Initial Cone
                .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .splineToConstantHeading(new Vector2d(44, -29),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristLeft())
                .splineTo(new Vector2d(51,24),Math.toRadians(-90))
                .lineTo(new Vector2d(55,18))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                // Turn to face stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFive())
                .splineToLinearHeading(new Pose2d(53.5, -3,Math.toRadians(-90)),Math.toRadians(-90))
                .lineTo(new Vector2d(53.5, 28))
                .build();
    }

    public void LHJStackToJunction(Pose2d pose)
    {
        lHJStackToJunction = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawClose())
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorHigh())
                .UNSTABLE_addTemporalMarkerOffset(1, () -> extras.wristRight())
                // Move backward to the pole
                .lineTo(new Vector2d(0, -27))
                .splineToLinearHeading(new Pose2d(8, -34,Math.toRadians(-90)),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.clawOpen())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.wristMiddle())
                .build();
    }

    public void LHJJunctionToStack4(Pose2d pose)
    {
        lHJJunctionToStack4 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorFour())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LHJJunctionToStack3(Pose2d pose)
    {
        lHJJunctionToStack3 = drive.trajectorySequenceBuilder(pose)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorThree())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LHJJunctionToStack2(Pose2d pose)
    {
        lHJJunctionToStack2 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorTwo())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LHJJunctionToStack1(Pose2d pose)
    {
        lHJJunctionToStack1 = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(-0.5, 3))
                .build();
    }

    public void LHJLeftPark(Pose2d pose)
    {
        lHJLeftPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, -2))
                .lineToLinearHeading(new Pose2d(-5, -2, Math.toRadians(-2)))
                .build();
    }
    public void LHJMiddlePark(Pose2d pose)
    {
        lHJMiddlePark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineTo(new Vector2d(0, -22))
                .lineToLinearHeading(new Pose2d(-5, -22, Math.toRadians(-2)))
                .build();
    }
    public void LHJRightPark(Pose2d pose)
    {
        lHJRightPark = drive.trajectorySequenceBuilder(pose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> extras.elevatorGround())
                .lineToLinearHeading(new Pose2d(-5, -47, Math.toRadians(-2)))
                .build();
    }
}
