package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.SampleMechanism;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -gamepadEx.getLeftX(),
                () -> -gamepadEx.getLeftY(),
                () -> -gamepadEx.getRightX());

        // sample for action and command synergy and binding
        SampleMechanism sampleMechanism = new SampleMechanism(hardwareMap);
        Set<Subsystem> subsystemSet = Stream.of(drive).collect(Collectors.toSet());
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ActionCommand(sampleMechanism.doSampleMechanismAction(), subsystemSet));

        //TODO: see if this runs perpetually
        // also we might not want to be creating a new packet in each loop
        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
        schedule(driveCommand);
    }


}
