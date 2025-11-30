package frc.robot.subsystems.vision;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Vision extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;

    private static final String LEFT_LL = "limelight-left";
    private static final String RIGHT_LL = "limelight-right";
    private Pose2d visionPose = new Pose2d();


    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Feed vision measurements into the drivetrain's pose estimator
        // System.out.println("periodic is running");
    
        processLimelight(LEFT_LL);
        processLimelight(RIGHT_LL);
    }

    private void processLimelight(String limelightName) {

        // // Get the full Limelight results
        //LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // // If no valid target, skip
        // if (!results.valid) return;


        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LEFT_LL);
        if (LimelightHelpers.validPoseEstimate(poseEstimate))
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.004, 0.004, 0.004));
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    @Logged
    public Pose2d getpPose2d() {
        return visionPose;
    }

    }

