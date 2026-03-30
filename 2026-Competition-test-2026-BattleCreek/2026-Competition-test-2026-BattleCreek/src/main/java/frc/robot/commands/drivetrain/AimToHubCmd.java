package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSys;

public class AimToHubCmd extends Command {

    private final SwerveSys swerveSys;

    private Translation2d targetTranslation;

    private Rotation2d targetHeading;

    private final ProfiledPIDController aimController;

    public AimToHubCmd(SwerveSys swerveSys) {
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
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            targetTranslation = FieldConstants.redAllianceHubPose;
        }
        else {
            targetTranslation = FieldConstants.blueAllianceHubPose;
        }
    }
    
    @Override
    public void execute() {
		Translation2d extrapolation = new Translation2d(
            swerveSys.getFieldRelativeVelocity().getX(),
            swerveSys.getFieldRelativeVelocity().getY());
    
        Translation2d extrapolatedTranslation = swerveSys.getPose().getTranslation().plus(extrapolation);
        Translation2d extrapolatedTargetOffset = targetTranslation.minus(extrapolatedTranslation);

        targetHeading = Rotation2d.fromRadians(MathUtil.angleModulus(extrapolatedTargetOffset.getAngle().getRadians()));
        
        SmartDashboard.putNumber("target heading deg", targetHeading.getDegrees());

        PPHolonomicDriveController.overrideRotationFeedback(() -> targetHeading.getRadians());

        
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
        }
        return false;
    }

}