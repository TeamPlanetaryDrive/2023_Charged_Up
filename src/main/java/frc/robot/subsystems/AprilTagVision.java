package frc.robot.subsystems;

import java.sql.PseudoColumnUsage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Vector;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
// import frc.robot.wrapper.QuaternionWrapper;

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
        // 4 works well, doesn't work that well for side views
        config.quadSigma = 4;
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
        // .25 works, not for very side view tho
        double epsilon = 0.9;
        if (Math.abs(aspectRatio - 1) < epsilon) {
            return true;
        }
        return false;
    }

    // public static double[] quatToEuler(Quaternion quat) {
    //     QuaternionWrapper wrapper = new QuaternionWrapper(quat);
    //     double w = wrapper.getW();
    //     Vector<N3> v = wrapper.getV();
    //     double x = v.get(0, 0);
    //     double y = v.get(1, 0);
    //     double z = v.get(2, 0);
    //     double ysqr = y * y;
    
    //     double roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + ysqr));
    //     double pitch = Math.asin(2 * (w * y - z * x));
    //     double yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (ysqr + z * z));
    
    //     return new double[] { roll, pitch, yaw };
    // }

    // private Point rotatePoint(double x, double y, double angle) {
    //     double x_new = x * Math.cos(angle) - y * Math.sin(angle);
    //     double y_new = x * Math.sin(angle) + y * Math.cos(angle);
    //     return new Point(x_new, y_new);
    // }

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
        Scalar xColor = new Scalar(255, 255, 0); // blue

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
                    if(!(detection.getId() < 0 || detection.getId() > 8) && isSquare(detection) && detection.getHamming() < 1) {
                        
                        tags.add(detection.getId());
                    
                        if(debug) {System.out.println("found id " + detection.getId());}
                        
                        var cx = detection.getCenterX();
                        var cy = detection.getCenterY();
                        double[] cornerst = detection.getCorners();
                        double width = Math.sqrt(Math.pow(cornerst[0] - cornerst[2], 2) + Math.pow(cornerst[1] - cornerst[3], 2));
                        double height = Math.sqrt(Math.pow(cornerst[2] - cornerst[4], 2) + Math.pow(cornerst[3] - cornerst[5], 2));
                        var ll = 10;

                    
                        Point[] corners = new Point[4];
                        for (int i = 0; i < 4; i++) {
                            corners[i] = new Point(detection.getCornerX(i), detection.getCornerY(i));
                        }
                        
                        Imgproc.line(mat, corners[0], corners[2], new Scalar(0, 0, 255), 2);
                        Imgproc.line(mat, corners[1], corners[3], new Scalar(0, 0, 255), 2);

                        for (var i = 0; i <= 3; i++) {
                            var j = (i + 1) % 4;
                            var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                            var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                            Imgproc.line(mat, pt1, pt2, outlineColor, 2);
                        }

                        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 2);
                        Transform3d pose = estimator.estimate(detection);
                
                        Quaternion quaternion = pose.getRotation().getQuaternion();

                        double length = 100;
                        double x = 2 * (quaternion.getX() * quaternion.getZ() - quaternion.getW() * quaternion.getY());
                        double y = 2 * (quaternion.getY() * quaternion.getZ() + quaternion.getW() * quaternion.getX());
                        double z = 1 - 2 * (quaternion.getX() * quaternion.getX() + quaternion.getY() * quaternion.getY());
                        Point end =  new Point(cx + x * length, cy + y * length);

                        // Draw the line on the image
                        Imgproc.arrowedLine(mat, new Point(cx, cy), end, new Scalar(255, 0, 0), 2);
                        //System.out.println("[DEBUG] Tag Detected: " + pose);


                        MatOfPoint matOfPoint = new MatOfPoint();
                        matOfPoint.fromArray(corners);

                        Point[] baseCorners = matOfPoint.toArray();

                        for (int i = 0; i <= 3; i++) {
                            Imgproc.line(mat, baseCorners[i],  new Point(baseCorners[i].x + x * width, baseCorners[i].y  + y * height), new Scalar(255, 0, 255), 2);
                        }
                        for (int i = 0; i < 3; i++) {
                            Imgproc.line(mat, new Point(baseCorners[i].x + x * width, baseCorners[i].y  + y * height),  new Point(baseCorners[i + 1].x + x * width, baseCorners[i + 1].y  + y * height), new Scalar(255, 0, 255), 2);
                            Imgproc.line(mat, new Point(baseCorners[0].x + x * width, baseCorners[0].y  + y * height),  new Point(baseCorners[3].x + x * width, baseCorners[3].y  + y * height), new Scalar(255, 0, 255), 2);
                        }

                    }
                }
            }

            SmartDashboard.putString("tag", tags.toString());
            video.putFrame(mat);
            
        }
    }
    

}
