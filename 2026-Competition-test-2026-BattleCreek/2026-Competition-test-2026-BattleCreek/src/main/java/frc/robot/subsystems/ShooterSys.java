package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.FieldConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class ShooterSys extends SubsystemBase {

    SparkFlex shooterMtr;
    RelativeEncoder shooterEnc;
    SparkClosedLoopController shooterController;
    SwerveSys swerveSys;

    public ShooterSys(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        
        shooterMtr = new SparkFlex(CANDevices.shooterMtrId, MotorType.kBrushless);
        shooterEnc = shooterMtr.getEncoder();
        shooterController = shooterMtr.getClosedLoopController();

        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig
            .kS(0)
            .kV(0.0175)
            .kA(0);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.stallLimitAmps, ShooterConstants.freeLimitAmps, ShooterConstants.maxRPM);
        shooterConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1)
            .uvwMeasurementPeriod(15);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD)
            .apply(ffConfig);

        shooterMtr.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterRPM(double rpm) {
        shooterController.setSetpoint(rpm, ControlType.kVelocity);
    }
    
    public void setShooterVolts(Voltage volts) {
        shooterMtr.setVoltage(volts);
    }

    public void setControllerVolts(double volts){
        shooterController.setSetpoint(volts, ControlType.kVoltage);
    }

    /**
     * Returns the shooter's current velocity in RPM as measured by the encoder.
     * @return Shooter velocity in RPM.
     */
    public double getShooterRPM() {
        return shooterEnc.getVelocity();
    }

    /**
     * Returns the shooter's current velocity in radians per second as measured by the encoder.
     * @return Shooter velocity in radians per second.
     */
    public double getVelocityRadPerSec() {
        return shooterEnc.getVelocity() * 2.0 * Math.PI / 60.0;
    }

    public void stop() {
        shooterController.setSetpoint(0, ControlType.kVelocity);
    }

        /**
     * Returns the planar (ground) distance from the shooter's exit point to the center
     * of the alliance hub in feet. Uses Limelight field pose (when valid) and falls back
     * to odometry (`swerveSys.getPose()`) otherwise.
     *
     * This handles the case where the robot isn't square to the hub by computing the
     * true Euclidean distance between the shooter's position and the hub center.
     */
    public double getDistanceCenterHub() {
        double meters = getPlanarDistanceToHubMeters();
        return Units.metersToFeet(meters);
    }

    /**
     * Compute the planar (XY) distance in meters between the shooter's exit point and the
     * hub center. This uses the Limelight's field pose if available and meets the
     * target-area threshold; otherwise it uses the drivetrain odometry pose.
     */
    public double getPlanarDistanceToHubMeters() {
        // Choose hub position by alliance
        final Translation2d hubPose;
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            hubPose = FieldConstants.redAllianceHubPose;
        }
        else {
            hubPose = FieldConstants.blueAllianceHubPose;
        }

        // Prefer Limelight pose when it is reporting a visible target with sufficient area.
        Pose2d robotPose2d;
        boolean limelightHasTarget = LimelightHelpers.getTV(VisionConstants.LimelightName) &&
            LimelightHelpers.getTA(VisionConstants.LimelightName) >= VisionConstants.targetAreaPercentThreshold;

        if (limelightHasTarget) {
            // Use the alliance-specific wpi pose the Limelight publishes
            robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.LimelightName);

            // If the limelight pose looks like an all-zero default, fall back to odometry
            if (robotPose2d.getTranslation().getX() == 0.0 && robotPose2d.getTranslation().getY() == 0.0) {
                robotPose2d = swerveSys.getPose();
            }
        } else {
            robotPose2d = swerveSys.getPose();
        }

        // Apply shooter offset (robot -> shooter) using the rotation of the robot so the offset
        // is in field coordinates.
        Transform2d shooterOffsetTransform = new Transform2d(
            new Translation2d(ShooterConstants.shooterOffsetXMeters, ShooterConstants.shooterOffsetYMeters),
            new Rotation2d(0.0));

        Pose2d shooterPose = robotPose2d.transformBy(shooterOffsetTransform);

        double dx = (shooterPose.getTranslation().getX() - hubPose.getX()) + swerveSys.getFieldRelativeVelocity().getX();
        double dy = (shooterPose.getTranslation().getY() - hubPose.getY()) + swerveSys.getFieldRelativeVelocity().getY();

        return Math.hypot(dx, dy);
    }

    public double desiredRPM() {

        if (RobotContainer.operatorController.x().getAsBoolean()) {
            return 6000;
        } else {
        return (306 * getDistanceCenterHub()) + 1936;
        }
    }

    @Override
    public void periodic() {

    }
    
}
