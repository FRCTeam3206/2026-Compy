package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private Matrix<N3, N1> curStdDevs;
  private final EstimateConsumer estConsumer;

  // Simulation
  private PhotonCameraSim simCamera;
  private VisionSystemSim simVision;
  @Logged private Pose3d lastEstimatedPose;

  public Vision(String cameraName, Transform3d robotToCamera, EstimateConsumer estConsumer) {
    this.camera = new PhotonCamera(cameraName);
    this.poseEstimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);
    this.estConsumer = estConsumer;

    if (Robot.isSimulation()) {
      simVision = new VisionSystemSim("main");
      simVision.addAprilTags(kTagLayout);
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(80));
      cameraProp.setCalibError(0.5, 0.1);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(10);
      simCamera = new PhotonCameraSim(camera, cameraProp);
      simVision.addCamera(simCamera, robotToCamera);

      simCamera.enableDrawWireframe(true);
    }
  }

  public void periodic() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var result : camera.getAllUnreadResults()) {
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimatedStdDevs(visionEst, result.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            (est) ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> getSimDebugField().getObject("VisionEstimation").setPoses());
      }

      visionEst.ifPresent(
          (est) ->
              estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs));
      if (!visionEst.isEmpty()) {
        this.lastEstimatedPose = visionEst.get().estimatedPose;
      }
    }
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    simVision.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) simVision.resetRobotPose(pose);
  }

  private void updateEstimatedStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      curStdDevs = kSingleTagStdDevs;
    } else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        curStdDevs = kSingleTagStdDevs;
      } else {
        avgDist /= numTags;
        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist) / 30);
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  public Field2d getSimDebugField() {
    if (Robot.isSimulation()) {
      return simVision.getDebugField();
    }
    return null;
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
