package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "a")
public class BasicTeleOp extends LinearOpMode
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
        double elevHeightMax = -3000;
        double slope;
        double elevatorEncoderCounts;

        //NormalizedRGBA colors = extras.colorSensor.getNormalizedColors();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extras.wristMiddle();
        extras.clawClose();

        waitForStart();


        while (!isStopRequested())
        {

            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevator1.getCurrentPosition() + extras.elevator2.getCurrentPosition()) / 2;
            elevMult = slope * elevatorEncoderCounts + 1;

            if (gamepad1.right_bumper)
            {
                speedMultiplier = 0.6;
            }
            else if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.4;
            }
            else
            {
                speedMultiplier = 0.75;
            }

            adjustedAngle = extras.adjustAngleForDriverPosition(drive.getRawExternalHeading(), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSideways = -gamepad1.left_stick_x * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            drive.setWeightedDrivePower(new Pose2d(stickForwardRotated, stickSidewaysRotated, -gamepad1.right_stick_x));
            drive.update();

            if (gamepad1.y)
            {
                //gp1_y_pressed = true;
            }
            else if (!gamepad1.y && gp1_y_pressed)
            {
                Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
                book.TeleOpPoleLeft(startPose);
                drive.setPoseEstimate(startPose);

                drive.followTrajectorySequence(book.teleOpPoleLeft);
            }

            if (gamepad1.a)
            {
                //gp1_a_pressed = true;
            }
            else if (!gamepad1.a && gp1_a_pressed)
            {
                Pose2d startPose = drive.getPoseEstimate();
                book.TeleOpConeLeft(startPose);

                drive.followTrajectorySequence(book.teleOpConeLeft);
            }

            // MANUAL ELEVATOR CONTROL- gamepad 2
            // don't go below the limit switch
            if((extras.elevatorLimit.isPressed())&&(gamepad2.left_stick_y > 0))
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;
            }
            // don't go above the max height
            else if((extras.elevator1.getCurrentPosition() < elevHeightMax) &&(gamepad2.left_stick_y < 0))
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;
            }
            // don't go too low if turret is turned
            else if((extras.elevator1.getCurrentPosition() > -1150) && (gamepad2.left_stick_y > 0) && (extras.wristPosition != ExtraOpModeFunctions.WristPosition.MIDDLE))
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;
            }
            else
            {
                if((gamepad2.left_stick_y == 0) && !elevatorStopped)
                {
                    extras.elevator1.setPower(0);
                    extras.elevator2.setPower(0);
                    elevatorStopped = true;

                    extras.elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extras.elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extras.elevator1.setTargetPosition(extras.elevator1.getCurrentPosition());
                    extras.elevator2.setTargetPosition(extras.elevator1.getCurrentPosition());
                    extras.elevator1.setPower(1.0);
                    extras.elevator2.setPower(1.0);

                }
                else if (gamepad2.left_stick_y != 0)
                {
                    extras.elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator1.setPower(elevMult * gamepad2.left_stick_y);
                    extras.elevator2.setPower(elevMult * gamepad2.left_stick_y);
                    elevatorStopped = false;
                }
            }

            if (gamepad2.right_stick_y < 0)
            {
                //gp2_right_stick_y_neg_pressed = true;
            }
            else if ((gamepad2.right_stick_y == 0) && (gp2_right_stick_y_neg_pressed))
            {
                extras.elevatorHigh();
                gp2_right_stick_y_neg_pressed = false;
            }

            if (gamepad2.right_stick_y > 0)
            {
                //gp2_right_stick_y_pos_pressed = true;
            }
            else if ((gamepad2.right_stick_y == 0) && (gp2_right_stick_y_pos_pressed))
            {
                extras.elevatorGround();
                gp2_right_stick_y_pos_pressed = false;
            }

            // RESET IMU
            if ((gamepad1.back) && (gamepad1.b))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();
                drive.IMUInit(hardwareMap);
            }

            // RESET IMU
            if ((gamepad2.back) && (gamepad2.y))
            {
                gp2_y_pressed = true;
            }
            else if (!gamepad2.y && gp2_y_pressed)
            {
                extras.initElevator();
                gp2_y_pressed = false;

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


            if (gamepad2.a)
            {
                gp2_a_pressed = true;
            }
            else if (!gamepad2.a && gp2_a_pressed)
            {
                extras.clawOpen();
                gp2_a_pressed = false;

            }

            if (gamepad2.b)
            {
                gp2_b_pressed = true;
            }
            else if (!gamepad2.b && gp2_b_pressed)
            {
                extras.clawClose();
                gp2_b_pressed = false;
            }

            // wrist left movements
            if(gamepad2.dpad_left)
            {
                gp2_dpad_left_pressed = true;
            }
            else if (!gamepad2.dpad_left && gp2_dpad_left_pressed)
            {
                gp2_dpad_left_pressed = false;
                switch(extras.wristPosition)
                {
                    case LEFT:
                        extras.wristMiddle();
                        break;

                    case MIDDLE:
                        extras.wristRight();
                        break;

                    case RIGHT:
                        break;
                }
            }

            // wrist right movements
            if(gamepad2.dpad_right)
            {
                gp2_dpad_right_pressed = true;
            }
            else if (!gamepad2.dpad_right && gp2_dpad_right_pressed)
            {
                gp2_dpad_right_pressed = false;
                switch(extras.wristPosition)
                {
                    case LEFT:
                        break;

                    case MIDDLE:
                        extras.wristLeft();
                        break;

                    case RIGHT:
                        extras.wristMiddle();
                        break;
                }
            }

            // elevator up movements
            if(gamepad2.dpad_up)
            {
                //gp2_dpad_up_pressed = true;
            }
            else if (!gamepad2.dpad_up && gp2_dpad_up_pressed)
            {
                gp2_dpad_up_pressed = false;
                switch(extras.elevatorPosition)
                {
                    case COLLECT:
                        extras.elevatorJunction();
                        break;

                    case GROUND:
                        extras.elevatorLow();
                        break;

                    case LOW:
                        extras.elevatorMiddle();
                        break;

                    case MIDDLE:
                        extras.elevatorHigh();
                        break;

                    case HIGH:
                        break;
                }
            }


            // elevator down movements
            if(gamepad2.dpad_down)
            {
                //gp2_dpad_down_pressed = true;
            }
            else if (!gamepad2.dpad_down && gp2_dpad_down_pressed)
            {
                gp2_dpad_down_pressed = false;
                switch(extras.elevatorPosition)
                {
                    case COLLECT:
                        break;

                    case GROUND:
                        extras.elevatorGround();
                        break;

                    case LOW:
                        extras.elevatorJunction();
                        break;

                    case MIDDLE:
                        extras.elevatorLow();
                        break;

                    case HIGH:
                        extras.elevatorMiddle();
                        break;
                }
            }

            //colors = extras.colorSensor.getNormalizedColors();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("elevator1 encoder counts: ", extras.elevator1.getCurrentPosition());
            telemetry.addData("elevator2 encoder counts: ", extras.elevator2.getCurrentPosition());
            telemetry.addData("elevator limit: ", extras.elevatorLimit.isPressed());


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