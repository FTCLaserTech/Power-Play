package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
@Disabled
public class RightHighJunction extends LinearOpMode
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

        book.RightHighJuncInitial(drive.getPoseEstimate());
        book.RHJStackToJunction(new Pose2d(0,0,Math.toRadians(-90)));
        book.RHJJunctionToStack4(book.rHJStackToJunction.end());
        book.RHJJunctionToStack3(book.rHJStackToJunction.end());
        book.RHJJunctionToStack2(book.rHJStackToJunction.end());
        book.RHJJunctionToStack1(book.rHJStackToJunction.end());
        book.RHJLeftPark(book.rHJStackToJunction.end());
        book.RHJMiddlePark(book.rHJStackToJunction.end());
        book.RHJRightPark(book.rHJStackToJunction.end());

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.rightHighJunction);
        //At the wall for cone 5
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rHJStackToJunction);
        drive.followTrajectorySequence(book.rHJJunctionToStack4);
        //At the wall for cone 4
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rHJStackToJunction);

        drive.followTrajectorySequence(book.rHJJunctionToStack3);
        //At the wall for cone 3
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rHJStackToJunction);
        drive.followTrajectorySequence(book.rHJJunctionToStack2);
        //At the wall for cone 2
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rHJStackToJunction);
        drive.followTrajectorySequence(book.rHJJunctionToStack1);
        //At the wall for cone 1
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(-90)));
        drive.followTrajectorySequence(book.rHJStackToJunction);
        //End of stacking 1+5

        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.rHJLeftPark);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.rHJMiddlePark);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.rHJRightPark);
                break;

        }


        sleep(10000);
    }
}