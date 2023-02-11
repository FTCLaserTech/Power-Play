package org.firstinspires.ftc.teamcode;

import  static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.Vuforia;
import com.vuforia.PIXEL_FORMAT;

import boofcv.struct.image.GrayU8;
import boofcv.android.ConvertBitmap;
import boofcv.abst.fiducial.QrCodeDetector;
import boofcv.alg.fiducial.qrcode.QrCode;
import boofcv.factory.fiducial.ConfigQrCode;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.abst.fiducial.MicroQrCodeDetector;
import boofcv.alg.fiducial.microqr.MicroQrCode;
import boofcv.factory.fiducial.ConfigMicroQrCode;


import java.util.List;

@Config
public class ExtraOpModeFunctions
{
    public enum Signal {ONE, TWO, THREE}
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum FieldSide {RED, BLUE}

    int numRed = 0;
    int numGreen = 0;
    int numBlue = 0;

    public enum WristPosition {LEFT, MIDDLE, RIGHT}
    public WristPosition wristPosition = WristPosition.MIDDLE;
    public enum ElevatorPosition {COLLECT, GROUND, LOW, MIDDLE, HIGH, TWO, THREE, FOUR, FIVE}
    public ElevatorPosition elevatorPosition = ElevatorPosition.COLLECT;

    public static final double PI = 3.14159265;
    public int target = 0;

    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    public Servo claw;
    public Servo wrist;
    public LinearOpMode localLop = null;

    public DcMotorEx elevator1;
    public DcMotorEx elevator2;

    public TouchSensor elevatorLimit;
    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode lop)
    {
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        localLop = lop;

        elevator1.setDirection(DcMotorEx.Direction.REVERSE);
        elevator1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //elevator1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setTargetPosition(0);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //elevator1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevator2.setDirection(DcMotorEx.Direction.FORWARD);
        elevator2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //elevator2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setTargetPosition(0);
        //elevator2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //elevator2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
        displayPattern();


        VuforiaLocalizer.Parameters parameters;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
    }

    public void initElevator()
    {
        elevator1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevator1.setPower(0.2);
        elevator2.setPower(0.2);
        localLop.sleep(600);
        elevator1.setPower(0);
        elevator2.setPower(0);

        localLop.telemetry.addData("Limit ", elevatorLimit.getValue());
        localLop.telemetry.update();

        localLop.sleep(100);

        elevator1.setPower(-0.1);
        elevator2.setPower(-0.1);

        while(!elevatorLimit.isPressed())
        {
            ;
        }

        elevator1.setPower(0);
        elevator2.setPower(0);

        elevator1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        localLop.sleep(250);

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevator1.setTargetPosition(20);
        elevator2.setTargetPosition(20);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);

        localLop.telemetry.addLine("Elevator Initialized!");
        localLop.telemetry.update();

    }

    public void clawOpen()
    {
        claw.setPosition(0.4);
    }

    public void clawClose()
    {
        claw.setPosition(0.485);
    }

    public void clawMove (int distance)
    {
        claw.setPosition(claw.getPosition() + distance);
    }

    public void wristLeft()
    {
        wrist.setPosition(1.0);
        wristPosition = wristPosition.LEFT;
    }

    public void wristMiddle()
    {
        wrist.setPosition(0.505);
        wristPosition = wristPosition.MIDDLE;
    }

    public void wristRight()
    {
        wrist.setPosition(0.0);
        wristPosition = wristPosition.RIGHT;
    }

    public void elevatorGround()
    {
        target = 0;
        elevatorPosition = elevatorPosition.COLLECT;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorJunction()
    {
        target = 20;
        elevatorPosition = elevatorPosition.GROUND;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorLow()
    {
        target = 1210;
        elevatorPosition = elevatorPosition.LOW;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorMiddle()
    {
        target = 2020;
        elevatorPosition = elevatorPosition.MIDDLE;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorHigh()
    {
        target = 2820;
        elevatorPosition = elevatorPosition.HIGH;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }


    public void elevatorTwo()
    {
        target = 130;
        elevatorPosition = elevatorPosition.TWO;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorThree()
    {
        target = 247;
        elevatorPosition = elevatorPosition.THREE;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorFour()
    {
        target = 344;
        elevatorPosition = elevatorPosition.FOUR;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorFive()
    {
        target = 473;
        elevatorPosition = elevatorPosition.FIVE;

        elevator1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
    }


    // CAMERA LOGIC

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AYkCgy7/////AAABmcVEWPZVAkr+qqRZ5GKKMtplRC79gsSR0agZEVe/znTU27Ffh0FtXPIGLOSGcu+OdpREriws8ksSpiZCvHpGc8cMP5JhNkjYOk71bfFphPQeGzxAqQr+0w4bsMkf4XHP1cXHVbaVP89ifVwqpnOLSm6Z7poTfguO8PMlHnoJIL6KEdnddmgKmQclRMFlerlVjcT55VFL4YAOetN7tbBZHcC4o/zGFgXdTfQWGNug7wHPvStMAArpFZUbSMEmHMdckbXgCCGCGVZw3qYQV9D3ALkAlwvPGQo+RXckMJ3kgk6trHnzxojWVfxsuflrcyDzorAmx+qn4Ei6R+HqxkrM7mSAgV45vyVlwN5GlyF7yv8g";


    private int signalLocation = 0;

    public Signal grabAndProcessImage(FieldSide fieldSide)
    {

        Signal signal = Signal.ONE;
        Image imageRGB565 = null;

        CameraDevice.getInstance().start();

        try
        {
            Frame frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); ++i)
            {
                Image image = frame.getImage(i);

                if (image.getFormat() == PIXEL_FORMAT.RGB565)
                {
                    imageRGB565 = image;

                    break;
                }
            }

            if (imageRGB565 != null)
            {
                // grab the image
                Bitmap input = Bitmap.createBitmap(imageRGB565.getWidth(), imageRGB565.getHeight(), Bitmap.Config.RGB_565);
                input.copyPixelsFromBuffer(imageRGB565.getPixels());

                GrayU8 gray = ConvertBitmap.bitmapToGray(input, (GrayU8)null, null);
                MicroQrCodeDetector<GrayU8> detector = FactoryFiducial.microqr(new ConfigMicroQrCode(), GrayU8.class);
                detector.process(gray);

                // Gets a list of all the qr codes it could successfully detect and decode
                List<MicroQrCode> detections = detector.getDetections();

                for (MicroQrCode qr : detections) {
                    if (qr.message.contentEquals("1"))
                    {
                        signalLocation = 1;
                        signal = signal.ONE;
                    }
                    else if (qr.message.contentEquals("2"))
                    {
                        signalLocation = 2;
                        signal = signal.TWO;
                    }
                    else
                    {
                        signalLocation = 3;
                        signal = signal.THREE;
                    }
                }
            }
        }
        catch (InterruptedException exc)
        {
            exc.printStackTrace();
        }

        return signal;
    }
}

