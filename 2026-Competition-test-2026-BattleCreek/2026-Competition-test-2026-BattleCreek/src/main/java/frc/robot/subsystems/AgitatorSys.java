package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.AgitatorConstants;

public class AgitatorSys extends SubsystemBase {
    
    private final SparkMax agitatorMtr;
    private final RelativeEncoder agitatorEnc;
    private final double maxRPM = AgitatorConstants.maxRPM;

    public AgitatorSys () {

        agitatorMtr = new SparkMax(CANDevices.agitatorMtrId, MotorType.kBrushless);
        agitatorEnc = agitatorMtr.getEncoder();


        SparkMaxConfig agitatorConfig = new SparkMaxConfig();
        agitatorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        agitatorConfig.encoder
            .positionConversionFactor(25)
            .velocityConversionFactor(25);

    }


    /**
     * Sets the agitator motor to the specified RPM. Positive RPMs should intake balls, while negative RPMs should outtake balls.
     */
    public void setAgitatorRPM(double rpm, boolean reverse) {
        double mtrSpeed = rpm / maxRPM;
        if(reverse == true) {
            agitatorMtr.set(-mtrSpeed);
        }else{
            agitatorMtr.set(mtrSpeed);
        }
    }

    /** Returns the current RPM of the agitator motor. */
    public void getAgitatorRPM() {
        agitatorEnc.getVelocity();
    }

    /** Stops the agitator motor.*/
    public void stop() {
        agitatorMtr.set(0);
    }

}
