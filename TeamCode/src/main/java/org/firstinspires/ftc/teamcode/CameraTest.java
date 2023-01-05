package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "a")
public class CameraTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);


        int IMUReset = 0;
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double speedMultiplier;

        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_right_stick_y_neg_pressed = false;
        boolean gp2_right_stick_y_pos_pressed = false;
        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;
        boolean gp2_y_pressed = false;
        boolean gp1_y_pressed = false;
        boolean gp1_a_pressed = false;

        boolean elevatorStopped = true;

        double elevMultMin = 0.5;
        double elevMult = 0;
        double elevHeightMax = 3100;
        double slope;
        double elevatorEncoderCounts;

        double currentAmps1;
        double currentAmps2;
        double maxAmps = 0;
        int numDangerAmps = 0;

        //NormalizedRGBA colors = extras.colorSensor.getNormalizedColors();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extras.wristMiddle();
        extras.clawClose();

        waitForStart();


        while (!isStopRequested())
        {
            currentAmps1 = extras.elevator1.getCurrent(CurrentUnit.AMPS);
            currentAmps2 = extras.elevator2.getCurrent(CurrentUnit.AMPS);

            if (currentAmps1 > maxAmps)
            {
                maxAmps = currentAmps1;
            }
            else if (currentAmps2 > maxAmps)
            {
                maxAmps = currentAmps2;
            }

            if (currentAmps1 > 7 || currentAmps2 > 7)
            {
                numDangerAmps += 1;
            }

/*
            if (getRuntime() >= 90 && getRuntime() <= 91)
            {
                extras.pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                extras.displayPattern();
            }
            else
            {
                if (colors.alpha > 0.35)
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    extras.displayPattern();
                }
                else
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                    extras.displayPattern();
                }
            }

 */

            //colors = extras.colorSensor.getNormalizedColors();

            ExtraOpModeFunctions.Signal coneColor = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("elevator1 encoder counts: ", extras.elevator1.getCurrentPosition());
            telemetry.addData("elevator2 encoder counts: ", extras.elevator2.getCurrentPosition());
            telemetry.addData("elevator limit: ", extras.elevatorLimit.isPressed());
            //telemetry.addLine();


            telemetry.addData("Cone Color: ", coneColor);
            telemetry.addData("Red_", extras.numRed);
            telemetry.addData("Green_", extras.numGreen);
            telemetry.addData("Blue_", extras.numBlue);


            telemetry.addData("Elevator 1 Current Voltage: ", currentAmps1);
            telemetry.addData("Elevator 2 Current Voltage: ", currentAmps2);
            telemetry.addData("Max Amps: ", maxAmps);
            telemetry.addData("Number of times amps was greater than 7: ", numDangerAmps);

            telemetry.addData("Elapsed Time: ", getRuntime());


            //telemetry.addLine()
                   // .addData("Red", "%.3f", colors.red)
                  //  .addData("Green", "%.3f", colors.green)
                   // .addData("Blue", "%.3f", colors.blue)
                 //   .addData("Alpha", "%.3f", colors.alpha);

            telemetry.update();
        }
    }
}