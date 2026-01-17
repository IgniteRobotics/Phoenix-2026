package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final String photonCameraName_FrontLeft = "FRONT LEFT CAMERA";
    public static final Transform3d photonCameraTransform_FrontLeft = new Transform3d(new Translation3d(0.31623, 0.2758, 0.235), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-20)));
    public static final PhotonCamera photonCamera_FrontLeft = new PhotonCamera(photonCameraName_FrontLeft);
    public static final PhotonPoseEstimator photonPoseEstimator_FrontLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform_FrontLeft);

    public static final String photonCameraName_FrontRight = "FRONT RIGHT CAMERA";
    public static final Transform3d photonCameraTransform_FrontRight = new Transform3d(new Translation3d(0.31623, -0.2758, 0.235), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(20)));
    public static final PhotonCamera photonCamera_FrontRight = new PhotonCamera(photonCameraName_FrontRight);
    public static final PhotonPoseEstimator photonPoseEstimator_FrontRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform_FrontRight);
    
    public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;
}
