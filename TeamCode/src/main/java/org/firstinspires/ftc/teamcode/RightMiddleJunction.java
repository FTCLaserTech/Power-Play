package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
//@Disabled
public class RightMiddleJunction extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        extras.wristMiddle();
        sleep(500);
        extras.initElevator();

        Pose2d poseEstimate = drive.getPoseEstimate();

        book.RightMedJuncInitial(drive.getPoseEstimate());
        book.RMJStackToJunction(new Pose2d(0,0,Math.toRadians(-90)));
        book.RMJJunctionToStack4(book.rMJStackToJunction.end());
        book.RMJJunctionToStack3(book.rMJStackToJunction.end());
        book.RMJJunctionToStack2(book.rMJStackToJunction.end());
        book.RMJJunctionToStack1(book.rMJStackToJunction.end());
        book.RMJLeftPark(book.rMJStackToJunction.end());
        book.RMJMiddlePark(book.rMJStackToJunction.end());
        book.RMJRightPark(book.rMJStackToJunction.end());

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.rightMediumJunction);
        //At the wall for cone 5
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rMJStackToJunction);
        drive.followTrajectorySequence(book.rMJJunctionToStack4);
        //At the wall for cone 4
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rMJStackToJunction);
        drive.followTrajectorySequence(book.rMJJunctionToStack3);
        //At the wall for cone 3
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rMJStackToJunction);
        drive.followTrajectorySequence(book.rMJJunctionToStack2);
        //At the wall for cone 2
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rMJStackToJunction);
        drive.followTrajectorySequence(book.rMJJunctionToStack1);
        //At the wall for cone 1
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rMJStackToJunction);
        //End of stacking 1+5

        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.rMJLeftPark);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.rMJMiddlePark);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.rMJRightPark);
                break;

        }


        sleep(10000);
    }
}