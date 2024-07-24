package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.SampleMechanism;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Config
@TeleOp(name = "SAMPLE", group = "TeleOp")
public class Blue_Red_TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driver station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)));
        DriveCommand driveCommand = new DriveCommand(
                drive,
                () -> -gamepadEx.getLeftX(),
                () -> -gamepadEx.getLeftY(),
                () -> -gamepadEx.getRightX()
        );
        //TODO: see if this runs perpetually
        // also we might not want to be creating a new packet in each loop
        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.drive.pose;
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
        schedule(driveCommand);

        //SampleMechanism sampleMechanism = new SampleMechanism(hardwareMap);
        //schedule(new ActionCommand(sampleMechanism.doSampleMechanismAction(), ));
    }
    /*
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMechanism sampleMechanism = new SampleMechanism(hardwareMap);
        // using ftcLib gamepadEx class for their key/button reader classes
        ArrayList<Action> runningActions = new ArrayList<>();

        waitForStart();
        while (opModeIsActive()) {
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
        }
    }
     */


}
