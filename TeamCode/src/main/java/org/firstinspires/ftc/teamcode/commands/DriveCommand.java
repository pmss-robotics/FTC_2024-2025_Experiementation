package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * The default command of {@link DriveSubsystem}
 */
public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier lx, ly, rx;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx) {
        this.drive = drive;
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        drive.driverCentric(lx.getAsDouble(), ly.getAsDouble(), rx.getAsDouble());
    }

}
