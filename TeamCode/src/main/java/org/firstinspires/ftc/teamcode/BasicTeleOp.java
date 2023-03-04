package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        boolean gp2_right_bumper_pressed = false;
        boolean gp2_right_trigger_pressed = false;
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

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extras.wristMiddle();
        extras.clawOpen();

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

            if (currentAmps1 >= 7 || currentAmps2 >= 7)
            {
                numDangerAmps += 1;

                telemetry.addLine("WARNING: HIGH AMPS");

                if(numDangerAmps >= 10)
                {
                    // in future, ONLY stop arm motion
                    stop();
                }
            }
            else
            {
                numDangerAmps = 0;
            }

            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevator1.getCurrentPosition() + extras.elevator2.getCurrentPosition()) / 2;
            elevMult = slope * elevatorEncoderCounts + 1;

            if (gamepad1.right_bumper)
            {
                speedMultiplier = 0.6 * elevMult;
            }
            else if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.4 * elevMult;
            }
            else
            {
                speedMultiplier = 0.75 * elevMult;
            }

            adjustedAngle = extras.adjustAngleForDriverPosition(drive.imu.getAngularOrientation().firstAngle, ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSideways = -gamepad1.left_stick_x * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            drive.setWeightedDrivePower(new Pose2d(stickForwardRotated, stickSidewaysRotated, -gamepad1.right_stick_x));
            drive.update();

            // MANUAL ELEVATOR CONTROL- gamepad 2
            // stop if the limit switch is pressed
            float elevatorStick = gamepad2.left_stick_y;
            if(extras.elevatorLimit.isPressed())
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;

                // it's OK to move up if the limit switch is pressed
                if(elevatorStick < 0)
                {
                    extras.elevator1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator1.setPower(-elevatorStick);
                    extras.elevator2.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
                else if (gamepad2.right_stick_y < 0)
                {
                    extras.elevatorHigh();
                }

            }
            // don't go above the max height
            else if((extras.elevator1.getCurrentPosition() > elevHeightMax) && (elevatorStick < 0))
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;

                int elevPos1 = extras.elevator1.getCurrentPosition();
                extras.elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator1.setTargetPosition(elevPos1);
                extras.elevator2.setTargetPosition(elevPos1);
                extras.elevator1.setPower(1.0);
                extras.elevator2.setPower(1.0);
            }

            // don't go too low if turret is turned
            else if((extras.elevator1.getCurrentPosition() < 1200) && (elevatorStick > 0) && (extras.wristPosition != ExtraOpModeFunctions.WristPosition.MIDDLE))
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
                elevatorStopped = true;

                int elevPos1 = extras.elevator1.getCurrentPosition();
                extras.elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator1.setTargetPosition(elevPos1);
                extras.elevator2.setTargetPosition(elevPos1);
                extras.elevator1.setPower(1.0);
                extras.elevator2.setPower(1.0);
            }
            else
            {
                // If stick is not moved, only set power to 0 once
                if((elevatorStick == 0) && !elevatorStopped)
                {
                    extras.elevator1.setPower(0);
                    extras.elevator2.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevator1.getCurrentPosition();
                    extras.elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevator1.setTargetPosition(elevPos1);
                    extras.elevator2.setTargetPosition(elevPos1);
                    extras.elevator1.setPower(1.0);
                    extras.elevator2.setPower(1.0);

                }
                else if (elevatorStick != 0)
                {
                    extras.elevator1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator1.setPower(-elevatorStick);
                    extras.elevator2.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
            }

            // Elevator to top with right stick up
            if (gamepad2.right_stick_y < 0)
            {
                extras.elevatorHigh();
            }

            // Elevator to bottom with right sitck down
            if ((gamepad2.right_stick_y > 0)  && (extras.wristPosition == ExtraOpModeFunctions.WristPosition.MIDDLE))
            {
                extras.elevatorGround();
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

            // Init Elevator
            if ((gamepad2.back) && (gamepad2.y))
            {
                gp2_y_pressed = true;
            }
            else if (!gamepad2.y && gp2_y_pressed)
            {
                extras.initElevator();
                gp2_y_pressed = false;

            }

            // Claw Open
            if (gamepad2.a)
            {
                gp2_a_pressed = true;
            }
            else if (!gamepad2.a && gp2_a_pressed)
            {
                extras.clawOpen();
                gp2_a_pressed = false;
            }

            // Claw Close
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
                       // extras.wristMiddle();
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
                       // extras.wristMiddle();
                        break;

                    case MIDDLE:
                        extras.wristLeft();
                        break;

                    case RIGHT:
                        extras.wristMiddle();
                        break;
                }
            }

            if (gamepad2.right_bumper)
            {
                gp2_right_bumper_pressed = true;
            }
            else if (!gamepad2.right_bumper && gp2_right_bumper_pressed)
            {
                gp2_right_bumper_pressed = false;
                extras.elevatorFive();
            }

            if(gamepad2.right_trigger > 0)
            {
                gp2_right_trigger_pressed = true;
            }
            else if ((gamepad2.right_trigger == 0) && gp2_right_trigger_pressed)
            {
                gp2_right_trigger_pressed = false;
                switch(extras.elevatorPosition)
                {
                    case FOUR:
                        extras.elevatorThree();
                        break;

                    case THREE:
                        extras.elevatorTwo();
                        break;

                    case TWO:
                        extras.elevatorGround();
                        break;

                    default:
                        extras.elevatorThree();
                        break;
                }
            }

            // DPAD elevator up movements
            if(gamepad2.dpad_up)
            {
                gp2_dpad_up_pressed = true;
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

            // DPAD elevator down movements
            if(gamepad2.dpad_down)
            {
                gp2_dpad_down_pressed = true;
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

            extras.setLeds(getRuntime());

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Robot x coordinate: ", poseEstimate.getX());
            telemetry.addData("Robot y coordinate: ", poseEstimate.getY());
            telemetry.addData("Robot heading: ", poseEstimate.getHeading());
            telemetry.addData("Elevator1 encoder counts: ", extras.elevator1.getCurrentPosition());
            telemetry.addData("Elevator2 encoder counts: ", extras.elevator2.getCurrentPosition());
            telemetry.addData("Elevator limit: ", extras.elevatorLimit.isPressed());
            telemetry.addData("Elevator stopped? ", elevatorStopped);
            telemetry.addLine();

            telemetry.addData("Elevator1 current voltage: ", currentAmps1);
            telemetry.addData("Elevator2 current voltage: ", currentAmps2);
            telemetry.addData("Max amps: ", maxAmps);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}