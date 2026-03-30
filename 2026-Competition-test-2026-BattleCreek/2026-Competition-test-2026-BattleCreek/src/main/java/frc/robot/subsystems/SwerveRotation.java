package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lightweight subsystem which represents ownership of the robot's rotational controller.
 *
 * PointCmd will require this subsystem instead of the full SwerveSys so driver translation
 * can continue while a separate command controls rotation. Internally this delegates to
 * the real {@link SwerveSys} which actually applies the omega override.
 */
public class SwerveRotation extends SubsystemBase {
    private final SwerveSys swerveSys;

    public SwerveRotation(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
    }

    /**
     * Set (or clear) the desired rotational override in radians/sec. Use Optional.empty() to clear.
     */
    public void setOmegaOverride(Optional<Double> omegaRadPerSec) {
        swerveSys.setOmegaOverrideRadPerSec(omegaRadPerSec);
    }

    public boolean hasOverride() {
        return swerveSys.hasOmegaOverride();
    }
}
