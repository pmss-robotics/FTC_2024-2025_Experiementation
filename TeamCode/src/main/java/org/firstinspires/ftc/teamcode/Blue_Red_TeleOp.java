package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.SampleMechanism;

import java.util.ArrayList;

@Config
@TeleOp(name="SAMPLE", group="TeleOp")
public class Blue_Red_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // data sent to telemetry shows up on dashboard and driver station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        SampleMechanism sampleMechanism = new SampleMechanism(hardwareMap);
        // using ftcLib gamepadEx class for their key/button reader classes
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        ArrayList<Action> runningActions = new ArrayList<>();

        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepadEx1.getLeftY(),
                            -gamepadEx1.getLeftX()
                    ),
                    -gamepadEx1.getRightX()
            ));
            drive.updatePoseEstimate();

            // example of how you'd queue up actions
            // for larger more complex motions, use SequentialAction and ParallelAction
            if(gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                runningActions.add(sampleMechanism.doSampleMechanismAction());
            }
            // loops through and adds actions back to runningActions if uncompleted
            ArrayList<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;



            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
