package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import boofcv.struct.image.GrayU8;
import boofcv.android.ConvertBitmap;
import boofcv.abst.fiducial.QrCodeDetector;
import boofcv.alg.fiducial.qrcode.QrCode;
import boofcv.factory.fiducial.ConfigQrCode;
import boofcv.factory.fiducial.FactoryFiducial;

import boofcv.abst.fiducial.MicroQrCodeDetector;
import boofcv.alg.fiducial.microqr.MicroQrCode;
import boofcv.factory.fiducial.ConfigMicroQrCode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(group = "a")
public class CameraTest extends LinearOpMode
{
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;
    private static final String VUFORIA_KEY =
            "AYkCgy7/////AAABmcVEWPZVAkr+qqRZ5GKKMtplRC79gsSR0agZEVe/znTU27Ffh0FtXPIGLOSGcu+OdpREriws8ksSpiZCvHpGc8cMP5JhNkjYOk71bfFphPQeGzxAqQr+0w4bsMkf4XHP1cXHVbaVP89ifVwqpnOLSm6Z7poTfguO8PMlHnoJIL6KEdnddmgKmQclRMFlerlVjcT55VFL4YAOetN7tbBZHcC4o/zGFgXdTfQWGNug7wHPvStMAArpFZUbSMEmHMdckbXgCCGCGVZw3qYQV9D3ALkAlwvPGQo+RXckMJ3kgk6trHnzxojWVfxsuflrcyDzorAmx+qn4Ei6R+HqxkrM7mSAgV45vyVlwN5GlyF7yv8g";

    private int signalLocation = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {

        // Open a webcam at a resolution close to 640x480
        //Webcam webcam = UtilWebcamCapture.openDefault(640,480);
        Image imageRGB565 = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters parameters;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);

        waitForStart();

        while (!isStopRequested())
        {
            CameraDevice.getInstance().start();
            signalLocation = 0;

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
                        // The message encoded in the marker
                        telemetry.addData("message: ", qr.message);

                        if (qr.message.contentEquals("1"))
                        {
                            signalLocation = 1;
                        }
                        else if (qr.message.contentEquals("2"))
                        {
                            signalLocation = 2;
                        }
                        else
                        {
                            signalLocation = 3;
                        }
                    }
                }
                telemetry.addData("signalLocation ", signalLocation);
            }
            catch (InterruptedException exc)
            {
                exc.printStackTrace();
            }

            telemetry.update();
        }
    }
}