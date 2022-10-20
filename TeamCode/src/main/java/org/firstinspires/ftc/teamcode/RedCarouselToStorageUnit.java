package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(group = "a")
public class RedCarouselToStorageUnit extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        sleep(1000);
        extras.initArm();

        telemetry.addLine("Initialized");
        telemetry.update();

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        //
        // scan for capstone
        ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);

        book.RedCarouselSpinDuck(drive.getPoseEstimate());
        drive.followTrajectorySequence(book.redCarouselSpinDuck);

        // Move to shipping hub - change depending on capstone location
        switch (markerPosition)
        {
            case RIGHT:
                book.RedRightBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.redRightBlockCarousel);
                break;
            case MIDDLE:
                book.RedMiddleBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.redMiddleBlockCarousel);
                break;
            case LEFT:
                book.RedLeftBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.redLeftBlockCarousel);
                break;
        }

    book.RedParkInStorageFromCarousel(drive.getPoseEstimate());
    drive.followTrajectorySequence(book.redParkInStorageFromCarousel);

    }
}