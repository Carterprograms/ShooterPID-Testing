package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class AlignToTrench extends Command {

    private final SwerveSys swerveSys;

    private Rotation2d targetHeading;

    private final ProfiledPIDController aimController;

    public AlignToTrench(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        aimController = new ProfiledPIDController(
            AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
            new Constraints(
                AutoConstants.autoAimTurnSpeedRadPerSec,
                AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        double currentRad = swerveSys.getHeading().getRadians();
        double diffTo0 = Math.abs(MathUtil.angleModulus(0.0 - currentRad));
        double diffToPi = Math.abs(MathUtil.angleModulus(Math.PI - currentRad));
        targetHeading = (diffTo0 <= diffToPi)
            ? new Rotation2d (0)
            : new Rotation2d (Math.PI); // Maybe change to Math.PI?
    }
    
    @Override
    public void execute() {
        double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getRadians(), targetHeading.getRadians());
        swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
	}

    @Override
    public void end(boolean isInterrupted) {
        swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
        // Provide a DoubleSupplier; use NaN to indicate "no override" (PPHolonomicDriveController
        // implementations commonly check for NaN to disable an override).
        PPHolonomicDriveController.overrideRotationFeedback(() -> Double.NaN);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(swerveSys.getHeading().getDegrees() - targetHeading.getDegrees()) > AutoConstants.autoAimToleranceDeg) {
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(0.0));
            return true;
        }else {
        return false;
        }
    }

}