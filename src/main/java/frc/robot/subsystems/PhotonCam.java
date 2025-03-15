package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonCam extends SubsystemBase {

    PhotonCamera aprilTag1 = new PhotonCamera("Apriltag_1");
    PhotonCamera aprilTag3 = new PhotonCamera("Apriltag_3");

    // location of camera on the robot
    public final Transform3d aprilTag1Pos = new Transform3d(0.3302, 0, 0.209, new Rotation3d());
    public final Transform3d aprilTag3Pos = new Transform3d(0.3302, 0, 0.209, new Rotation3d());
    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);//TODO: measure
    final double TARGET_HEIGHT_METERS = 0.3;//TODO: verify
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0); // TODO: verify

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            aprilTag1Pos);

    public PhotonCam() {
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        Optional<EstimatedRobotPose> robotPose = Optional.empty();

        for (PhotonPipelineResult change : aprilTag1.getAllUnreadResults()) {
            robotPose = poseEstimator.update(change);
        }

        return robotPose;
    }

    public double getY1TranslationFromTarget(){
        double yTranslation = Double.MAX_VALUE;
        var results = aprilTag1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));
                Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-target.getYaw()));
                yTranslation = translation.getY();
            }
        }
        return yTranslation;
    }

    public double getCamera1Yaw(){
        double yaw = Double.MAX_VALUE;
        var results = aprilTag1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                yaw =  target.getYaw();
            }
        }
        return yaw;
    }
    public double getY2TranslationFromTarget(){
        double yTranslation = Double.MAX_VALUE;
        var results = aprilTag3.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));
                Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-target.getYaw()));
                yTranslation = translation.getY();
            }
        }
        return yTranslation;
    }
    public double getCamera3Yaw(){
        double yaw = Double.MAX_VALUE;
        var results = aprilTag1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                yaw =  target.getYaw();
            }
        }
        return yaw;
    }
    // puts coral/Algae status on smartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("AprilTag_1 yaw ", getCamera1Yaw());
        SmartDashboard.putNumber("AprilTag y translation (TODO: measure/verify) ", getY1TranslationFromTarget());
    }

}