package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    public MecanumDrive drive;
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
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                       input.x,
                        input.y
                ),
                rx
        ));
        drive.updatePoseEstimate();
    }

    public void driverCentric(double lx, double ly, double rx) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        lx,
                        ly
                ),
                rx
        ));
        drive.updatePoseEstimate();
    }
}
