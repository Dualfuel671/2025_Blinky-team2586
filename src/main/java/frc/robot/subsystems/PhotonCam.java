package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class PhotonCam extends SubsystemBase {

    PhotonCamera aprilTag1 = new PhotonCamera("Apriltag_1");

    // location of camera on the robot
    public final Transform3d aprilTag1Pos = new Transform3d(0.3302, 0, 0.209, new Rotation3d());

    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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

}