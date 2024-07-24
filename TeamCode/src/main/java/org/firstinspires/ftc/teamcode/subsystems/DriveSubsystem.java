package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    public DriveSubsystem(MecanumDrive drive) {
        this.drive = drive;
    }
    // TODO: see if driver centric is already robot centric and if not, ummm.
    public void robotCentric(double lx, double ly, double rx) {
        Pose2d poseEstimate = drive.pose;
        Vector2d input = new Vector2d(ly, lx);
        // we probably need to rotate the input vector by poseEstimate.heading
        // idk how and I don't want to just slam a formula in directly
        // maybe there's a method somewhere otherwise we'll need to do that.
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                       input.x,
                        input.y
                ),
                rx
        ));
        updatePoseEstimate();
    }

    public void driverCentric(double lx, double ly, double rx) {
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        lx,
                        ly
                ),
                rx
        ));
        updatePoseEstimate();
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        drive.setDrivePowers(powers);
    }

    public PoseVelocity2d updatePoseEstimate() {
        return drive.updatePoseEstimate();
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return drive.actionBuilder(beginPose);
    }

    public Pose2d getPose() {
        return drive.pose;
    }
}
