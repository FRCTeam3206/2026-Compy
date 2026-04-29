package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(kCameraName);
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(kTagLayout);
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(80));
            cameraProp.setCalibError(0.35, 0.1);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, kRobotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
        }
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
        
    } 
}
