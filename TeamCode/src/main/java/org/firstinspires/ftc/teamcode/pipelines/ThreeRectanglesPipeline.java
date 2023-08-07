package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ThreeRectanglesPipeline extends OpenCvPipeline {

    public Scalar nonSelectedColor = new Scalar(0, 255,0);
    public Scalar selectedColor = new Scalar(0,0,255);

    public Rect rect1 = new Rect(110,42,40,40);
    public Rect rect2 = new Rect(160,42,40,40);
    public Rect rect3 = new Rect(210,42,40,40);
    public int selectedRect = -1;

    @Override
    public Mat processFrame(Mat input){
        drawRectangles(input);

        return input;
    }

    public void drawRectangles(Mat input){
        Imgproc.rectangle(input, rect1, nonSelectedColor);
        Imgproc.rectangle(input, rect2, nonSelectedColor);
        Imgproc.rectangle(input, rect3, nonSelectedColor);

        switch (selectedRect){
            case 1:
                Imgproc.rectangle(input, rect1, selectedColor);
                break;
            case 2:
                Imgproc.rectangle(input, rect2, selectedColor);
                break;
            case 3:
                Imgproc.rectangle(input, rect3, selectedColor);
                break;
        }
    }
}
