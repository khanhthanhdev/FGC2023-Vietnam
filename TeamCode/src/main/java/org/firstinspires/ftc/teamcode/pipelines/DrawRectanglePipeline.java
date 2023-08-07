package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DrawRectanglePipeline extends OpenCvPipeline {

    public Scalar nonSelectedColor = new Scalar(0,255,0);

    public Rect rect1 = new Rect(20,120,50,50);

    @Override
    public Mat processFrame(Mat input){
        Imgproc.rectangle(input, rect1, nonSelectedColor);
        return input;
    }
}
