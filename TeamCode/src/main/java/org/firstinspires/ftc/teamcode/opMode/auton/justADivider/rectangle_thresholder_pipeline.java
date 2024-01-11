package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class rectangle_thresholder_pipeline extends OpenCvPipeline {
    private String location = "nothing"; // output
    public Scalar lower = new Scalar(0, 0, 0); // HSV threshold bounds
    public Scalar upper = new Scalar(255, 255, 255);

    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // imamge analyzed after thresholding
    private Mat maskedInputMat = new Mat();

    // Rectangle regions to be scanned
    private Point topLeft1, bottomRight1;
    private Point topLeft2, bottomRight2;

    public rectangle_thresholder_pipeline(Point TL1, Point BR1, Point TL2, Point BR2) {
        this.topLeft1 = TL1;
        this.bottomRight1 = BR1;
        this.topLeft2 = TL2;
        this.bottomRight2 = BR2;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert from BGR to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lower, upper, binaryMat);


        // Scan both rectangle regions, keeping track of how many
        // pixels meet the threshold value, indicated by the color white
        // in the binary image
        double w1 = 0, w2 = 0;
        // process the pixel value for each rectangle  (255 = W, 0 = B)
        for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
            for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w1++;
                }
            }
        }

        for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
            for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w2++;
                }
            }
        }

        // Determine object location
        if (w1 > w2) {
            location = "1";
        } else if (w1 < w2) {
            location = "2";
        }

        return binaryMat;
    }

    public String getLocation() {
        return location;
    }
}
