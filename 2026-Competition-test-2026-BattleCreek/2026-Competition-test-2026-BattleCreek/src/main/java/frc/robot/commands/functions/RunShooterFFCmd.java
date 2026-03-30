package frc.robot.commands.functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSys;

public class RunShooterFFCmd extends Command {
    private final ShooterSys shooter;
    private final SimpleMotorFeedforward ff;
    private final PIDController pid;
    private final double targetRPM;

    // require shooter so command has exclusive control
    public RunShooterFFCmd(ShooterSys shooter, double targetRPM) {
        this.shooter = shooter;
        // ff constants: ks, kv, ka. Ensure kv/ka units match rad/s (see note below)
        this.ff = new SimpleMotorFeedforward(0, 0.0175, 0);
        // PID controller works in rad/s
        this.pid = new PIDController(0, 0, 0);
        this.targetRPM = targetRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        // convert RPM -> rad/s
        double targetRadPerSec = targetRPM * 2.0 * Math.PI / 60.0;

        double currentRadPerSec = shooter.getVelocityRadPerSec();

        // Feedforward (volts) for target vel
        double ffVolts = ff.calculate(targetRadPerSec);

        double pidoffsetRPM = pid.calculate(currentRadPerSec, targetRadPerSec);

        // write voltage via motor controller (voltage control)
        shooter.setControllerVolts(pidoffsetRPM + ffVolts);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted; change to finish when on-target if you want
    }
}
