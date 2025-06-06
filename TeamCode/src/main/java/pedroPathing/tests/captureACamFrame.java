package pedroPathing.tests;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@TeleOp(name = "Dashboard Photo Capture", group = "Linear Opmode")
@Config
public class captureACamFrame extends LinearOpMode {

    OpenCvWebcam camera;
    PhotoCapturePipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        pipeline = new PhotoCapturePipeline();
        camera.setPipeline(pipeline);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 30); // 0 ms delay between frames

        camera.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pipeline.triggerPhoto();
                telemetry.addLine("Photo capture triggered");
            }
            telemetry.update();
            sleep(100);
        }

        camera.stopStreaming();
    }

    public static class PhotoCapturePipeline extends OpenCvPipeline {
        private volatile boolean takePhoto = false;

        public void triggerPhoto() {
            takePhoto = true;
        }

        @Override
        public Mat processFrame(Mat input) {
            if (takePhoto) {
                takePhoto = false;
                saveMatToDisk(input, "captured_photo.png");
            }
            return input;
        }

        public void saveMatToDisk(Mat mat, String filename) {
            Bitmap bmp = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, bmp);

            File path = Environment.getExternalStorageDirectory();
            File file = new File(path, filename);

            try {
                FileOutputStream out = new FileOutputStream(file);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
                out.close();
                Log.i("PhotoCapture", "Saved image to: " + file.getAbsolutePath());
            } catch (IOException e) {
                Log.e("PhotoCapture", "Failed to save image", e);
            }
        }
    }
}
