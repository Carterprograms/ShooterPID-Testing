package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSys;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCmd extends Command {
        private final ShooterSys shooter;
        private final Timer timer;
        private final double duration; // seconds
        private final SimpleMotorFeedforward ff;

    public AutoShootCmd(ShooterSys shooter, double duration) {
        this.shooter = shooter;
        this.duration = duration;
        // ff constants: ks, kv, ka. Ensure kv/ka units match rad/s (see note below)
        this.ff = new SimpleMotorFeedforward(0, 0.0175, 0);
        addRequirements(shooter);
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // convert RPM -> rad/s
        double targetRadPerSec = shooter.desiredRPM() * 2.0 * Math.PI / 60.0;
        //double measuredRPM = shooter.getShooterRPM();
        //double measuredRadPerSec = measuredRPM * 2.0 * Math.PI / 60.0;

        // Feedforward (volts) for target vel
        double ffVolts = ff.calculate(targetRadPerSec);

        double volts = ffVolts;

        // clamp to battery safe range
        double maxV = RobotController.getBatteryVoltage();
        volts = MathUtil.clamp(volts, -maxV, maxV);

        // write voltage via motor controller (voltage control)
        shooter.setControllerVolts(volts);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
