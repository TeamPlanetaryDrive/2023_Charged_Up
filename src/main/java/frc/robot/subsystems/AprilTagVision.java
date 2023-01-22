package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AprilTagVision extends SubsystemBase {

    public CvSink cvSink;
    public AprilTagDetector detector = new AprilTagDetector();
    public AprilTagDetector.Config config = new AprilTagDetector.Config();
    //clear counter goes up until it hits the threshold, then everything in the thread is removed
    private int clearCounter = 0;
    private int clearThreshold = 10000;
    // camera focal lens stuff, dont touch.
    public double fx = 699.3778103158814, fy = 677.716, cx = 345.61, cy = 207.13;

    // apriltag poseestimation, 0.1524 = 6in
    public AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(0.1524d, fx, fy, cx, cy);
    public AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);
   
    
    public GenericPublisher aprilTagInfo;
    private boolean debug = false;

    public AprilTagVision(){
        super();

        config.quadDecimate = 0;
        config.quadSigma = 2;
        config.numThreads = 4;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        aprilTagInfo = table.getTopic("apriltags").genericPublish(getName());

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        
        // apriltag stuff, see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/apriltag/AprilTagDetector.html
        detector.addFamily("tag16h5");
        detector.setConfig(config);
        // run on new thread
        Thread vThread = new Thread(() -> tagDetection());
        vThread.setDaemon(true);
        vThread.start();

       
    }

    public String getName() {
        return AprilTagDetection.class.getName();
    }

    private boolean isSquare(AprilTagDetection detection) {
        double[] corners = detection.getCorners();
        double width = Math.sqrt(Math.pow(corners[0] - corners[2], 2) + Math.pow(corners[1] - corners[3], 2));
        double height = Math.sqrt(Math.pow(corners[2] - corners[4], 2) + Math.pow(corners[3] - corners[5], 2));
        double aspectRatio = width / height;
        double epsilon = 0.3;
        if (Math.abs(aspectRatio - 1) < epsilon) {
            return true;
        }
        return false;
    }

    void tagDetection() {
        
        // setup camera & video
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(RobotMap.CAM_WID, RobotMap.CAM_HEI);

        cvSink = CameraServer.getVideo();
        CvSource video = CameraServer.putVideo("April Tag Detection", RobotMap.CAM_WID, RobotMap.CAM_HEI);

        // mat = matrix, very memory expensive, please reuse
        Mat mat = new Mat();
        Mat grayMat = new Mat();
        ArrayList<Integer> tags = new ArrayList<>();
        // colors for video
        Scalar outlineColor = new Scalar(0, 255, 0); // green
        Scalar xColor = new Scalar(0, 0, 255); // blue

        // can never be true, allows us to stop or restart bot
        while (!Thread.interrupted()) {

            // tells CvSink to grab frame and put it into source mat
            // == 0 if there is error
            if(cvSink.grabFrame(mat) == 0) {
                video.notifyError(cvSink.getError());
                continue;
            }    
           
            // convert mat to grayscalle
            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY);

            AprilTagDetection[] rawdetections = detector.detect(grayMat);
            ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();
            Collections.addAll(detections, rawdetections);
           
            clearCounter++;
            if(clearCounter >= clearThreshold)
            {
                tags.clear();
            }
                
           if(Collections.frequency(tags, 1) > 10) 
           {
                System.out.print("red community 1");
                tags.clear();
           }
            // check if detected, otherwise do nothin
            if (detections.size() != 0) {
                
                
                for (AprilTagDetection detection : detections) {
                    //kept at zero to make sure it's able to detect non frc apriltags
                    if(!(detection.getId() < 0 || detection.getId() > 8) && isSquare(detection)) {
                        
                        tags.add(detection.getId());
                    
                        if(debug) {System.out.println("found id " + detection.getId());}
                            
                        
                        for (var i = 0; i <= 3; i++) {
                            var j = (i + 1) % 4;
                            var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                            var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                            Imgproc.line(mat, pt1, pt2, outlineColor, 2);
                        }
        
                        var cx = detection.getCenterX();
                        var cy = detection.getCenterY();
                        var ll = 10;
                        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
                        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
                        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
                        
                    }

                }
            }

            // mat.release();
            // grayMat.release();

            SmartDashboard.putString("tag", tags.toString());
            video.putFrame(mat);
            
        }
    }

}
