/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package pedroPathing.tests;

import static pedroPathing.newOld.PositionStorage.intakeTargetPos;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import pedroPathing.AutoPIDS.ControlMotor;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Vision Color-Locator Teleop TestingWITHSERVO", group = "Concept")
//@Disabled
public class ConceptVisionColorLocatorTeleopTestingWithServo extends LinearOpMode
{
    public static final ColorRange FTC_YELLOW = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 220, 220,   0),  //yellow
            new Scalar(255, 255, 255)
    );
    public static final ColorRange FTC_RED = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 215, 0,  0),  //red
            new Scalar(255, 219, 180)
    );
    public static final ColorRange FTC_BLUE = new ColorRange(
            ColorSpace.RGB,
            new Scalar( 16,   0, 215), //blue
            new Scalar(255, 127, 255)
    );
    double intakeMotorPower = 0;
    int lateralTargetPos = 0;
    int verticalTargetPos = 0;
    int targetAngle =0;

    public static double multiplyValue = 1.08;
    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */

        ColorBlobLocatorProcessor colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)// Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        ColorBlobLocatorProcessor colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();
        ColorBlobLocatorProcessor colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(FTC_YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .setErodeSize(2)
                .build();

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorBlue)
                .addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setShowStatsOverlay(false)
                .build();

        Servo outakeArmServo = hardwareMap.get(Servo.class, "outakeArmServo");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakemotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ControlMotor intakeControlMotor = new ControlMotor();
        //outakeArmServo.setPosition((double) 144 / 360);
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        dashboardTelemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        dashboardTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
            blobs.addAll(colorLocatorRed.getBlobs());
            blobs.addAll(colorLocatorBlue.getBlobs());
            blobs.addAll(colorLocatorYellow.getBlobs());
            AtomicInteger i = new AtomicInteger(1);
            colorLocatorRed.getBlobs().forEach( blob -> {
                if(blob.getBoxFit() != null) {
                    //telemetry.addData("Red Blob " + i + " Size Y", blob.getBoxFit().size.height);
                    //telemetry.addData("Red Blob " + i + " Size X", blob.getBoxFit().size.width);
                    //telemetry.addData("Red Blob " + i + " Angle", blob.getBoxFit().angle);
                    i.getAndIncrement();
                }
            });
            AtomicInteger j = new AtomicInteger(1);
            colorLocatorYellow.getBlobs().forEach( blob -> {
                if(blob.getBoxFit() != null) {

                    //telemetry.addData();
                    //telemetry.addData("Yellow Blob " + j + " Size X", blob.getBoxFit().size.width);
                    //telemetry.addData("Yellow Blob " + j + " Angle", blob.getBoxFit().angle);
                    j.getAndIncrement();
                }
            });
            AtomicInteger k = new AtomicInteger(1);
            colorLocatorBlue.getBlobs().forEach( blob -> {
                if(blob.getBoxFit() != null) {
                    //telemetry.addData("Blue Blob " + k + " Size Y", blob.getBoxFit().size.height);
                    //telemetry.addData("Blue Blob " + k + " Size X", blob.getBoxFit().size.width);
                    //telemetry.addData("Blue Blob " + k + " Angle", blob.getBoxFit().angle);
                    k.getAndIncrement();
                }
            });

            /*
             * The list of Blobs can be filtered to remove unwanted Blobs.
             *   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
             *          conditions will remain in the current list of "blobs".  Multiple filters may be used.
             *
             * Use any of the following filters.
             *
             * ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
             *   Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
             *
             * ColorBlobLocatorProcessor.Util.filterByDensity(minDensity, maxDensity, blobs);
             *   A blob's density is an indication of how "full" the contour is.
             *   If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
             *   The density is the ratio of Contour-area to Convex Hull-area.
             *
             * ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
             *   A perfect Square has an aspect ratio of 1.  All others are > 1
             */
            //ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, blobs);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.filterByArea(300, 10000, blobs);  // filter out very small blobs.
            /*
             * The list of Blobs can be sorted using the same Blob attributes as listed above.
             * No more than one sort call should be made.  Sorting can use ascending or descending order.
             *     ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);      // Default
             *     ColorBlobLocatorProcessor.Util.sortByDensity(SortOrder.DESCENDING, blobs);
             *     ColorBlobLocatorProcessor.Util.sortByAspectRatio(SortOrder.DESCENDING, blobs);
             */

            telemetry.addLine(" Area Density Aspect  Center Angle");
            dashboardTelemetry.addLine(" Area Density Aspect  Center Angle");

            // Display the size (area) and center location for each Blob.
            ColorBlobLocatorProcessor.Blob selectedBlob = null; // Variable to hold the blob with the maximum y coordinate
            int minDistance = Integer.MAX_VALUE; // Initialize maxY to the smallest possible integer

            int xCenter = 320;
            int yCenter = 480;

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                String color = "Cat";
                if (colorLocatorRed.getBlobs().contains(b))
                    color = "Red";
                else if (colorLocatorBlue.getBlobs().contains(b))
                    color = "Blue";
                else if (colorLocatorYellow.getBlobs().contains(b))
                    color = "Yellow";
                else
                    continue;

                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

                dashboardTelemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y) + " " + b.getBoxFit().angle + " " + color);

                int xDistance = (int) boxFit.center.x - xCenter;
                int yDistance = (int) boxFit.center.y - yCenter;
                int distance = xDistance*xDistance + yDistance*yDistance;

                // Check if the current blob has a higher y coordinate than the current maxY
                if (distance < minDistance) {
                    minDistance = distance; // Update maxY
                    selectedBlob = b; // Update selectedBlob to the current blob
                }
            }
            telemetry.addLine(" ");
            telemetry.addLine(" ");
            dashboardTelemetry.addLine(" ");
            dashboardTelemetry.addLine(" ");
// After the loop, selectedBlob will hold the blob with the maximum y coordinate
            if (selectedBlob != null) {
                String color = "Cat";
                if (colorLocatorRed.getBlobs().contains(selectedBlob))
                    color = "Red";
                if (colorLocatorBlue.getBlobs().contains(selectedBlob))
                    color = "Blue";
                if (colorLocatorYellow.getBlobs().contains(selectedBlob))
                    color = "Yellow";
                // You can now perform actions with the selectedBlob
                // For example, you might want to log its details or process it further
                RotatedRect selectedBoxFit = selectedBlob.getBoxFit();
                lateralTargetPos = (int) Math.abs(selectedBoxFit.center.x-240);
                verticalTargetPos = (int)  Math.abs(480-selectedBoxFit.center.y);
                verticalTargetPos = (int) Math.pow(verticalTargetPos,multiplyValue);
                targetAngle = (int) selectedBlob.getBoxFit().angle;
                telemetry.addLine("Selected Blob: " + selectedBlob.getContourArea() + " at (" + (int) selectedBoxFit.center.x + ", " + (int) selectedBoxFit.center.y + ")" + " " + selectedBlob.getBoxFit().angle + " " + color);
                dashboardTelemetry.addLine("Selected Blob: " + selectedBlob.getContourArea() + " at (" + (int) selectedBoxFit.center.x + ", " + (int) selectedBoxFit.center.y + ")" + " " + selectedBlob.getBoxFit().angle + " " + color);
                telemetry.addLine("target:  X" + lateralTargetPos + " Y " + verticalTargetPos + " angle " + targetAngle);
                dashboardTelemetry.addLine("target:  X" + lateralTargetPos + " Y " + verticalTargetPos + " angle " + targetAngle);
            }


            if(gamepad1.a && verticalTargetPos < 500)
                intakeTargetPos = verticalTargetPos;
            if(gamepad1.b)
                intakeTargetPos=0;

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            FtcDashboard.getInstance().startCameraStream(portal, 30);
            dashboardTelemetry.update();
            telemetry.update();


            //intakeMotorPower = intakeControlMotor.PIDControl(intakeTargetPos+intakeTargetPosAdder, intakeMotor.getCurrentPosition());
            //intakeMotor.setPower(intakeMotorPower);

            sleep(50);
        }
    }
}
