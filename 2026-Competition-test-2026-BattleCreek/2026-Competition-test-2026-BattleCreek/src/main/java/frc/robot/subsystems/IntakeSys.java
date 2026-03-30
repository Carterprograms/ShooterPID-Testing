package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.CANDevices;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakeSys extends SubsystemBase {

    private final SparkMax rollerMtr;
    private final RelativeEncoder rollerEnc;

    public IntakeSys() {
        
        rollerMtr = new SparkMax(CANDevices.rollerMtrId, MotorType.kBrushless);
        rollerEnc = rollerMtr.getEncoder();

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(RollerConstants.stallLimitAmps, RollerConstants.freeLimitAmps, RollerConstants.maxRPM);
        rollerConfig.encoder
            .positionConversionFactor(0.33)
            .velocityConversionFactor(20);
        rollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(RollerConstants.rollerkP, RollerConstants.rollerkI, RollerConstants.rollerkD);

        rollerMtr.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Sets the roller motor to the specified RPM. Positive RPMs should intake balls, while negative RPMs should outtake balls.
     * @param rpm
     */
    public void setRollerRPM(double speed) {
        rollerMtr.set(speed);
    }

    /**
     * A method to get the current RPM of the roller motor.
     * @return The current RPM of the roller motor.
     */
    public double getRollerRPM() {
        return rollerEnc.getVelocity();
    }

    /**
     * Stops the roller motor.
     */
    public void stop() {
        rollerMtr.set(0);
    }
    
    /**
     * Returns the current (in amps) being drawn by the roller motor.
     * @return The current being drawn by the roller motor in amps.
     */
    public double getIntakeAmps() {
        return rollerMtr.getOutputCurrent();
    }

    /**
     * Returns the current temperature of the roller motor in degrees Celsius.
     * @return The current temperature of the roller motor in degrees Celsius.
     */
    public double getIntakeTemp() {
        return rollerMtr.getMotorTemperature();
    }

}
