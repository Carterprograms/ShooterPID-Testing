package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveRotation;
import frc.robot.util.limelight.LimelightHelpers;
import java.util.Optional;
import frc.robot.Constants.DriveConstants;

public class PointCmd extends Command {
    private final SwerveRotation swerveRotation;
    private final double kP = 10; // Proportional constant for centering (deg per deg)
    private final double tolerance = 4; // Degrees of error tolerance

    public PointCmd(SwerveRotation swerveRotation) {
        this.swerveRotation = swerveRotation;
        addRequirements(swerveRotation);
    }

    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("limelight"); // degrees

        // Proportional controller (deg -> deg/sec)
        double rotationDegPerSec = kP * tx;

        // Convert to radians/sec
        double rotationRadPerSec = Math.toRadians(rotationDegPerSec);

        // Clamp to +/- maxTurnSpeedRadPerSec
        rotationRadPerSec = Math.max(-94,
                                     Math.min(DriveConstants.maxTurnSpeedRadPerSec, rotationRadPerSec));

        // Set the omega override (invert if your drive expects opposite sign)
        swerveRotation.setOmegaOverride(Optional.of(-rotationRadPerSec));
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isTeleop()) {
            return false;
        }
        return Math.abs(LimelightHelpers.getTX("limelight")) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        swerveRotation.setOmegaOverride(Optional.empty());
    }
}