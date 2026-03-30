package frc.robot.commands.functions;

import frc.robot.subsystems.ShooterSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCmd extends Command {
    
    private final ShooterSys shooterSys;
    private final double targetRPM;

    public ShootCmd(ShooterSys shooterSys, double targetRPM) {
        this.shooterSys = shooterSys;
        this.targetRPM = targetRPM;

        addRequirements(shooterSys);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSys.setShooterRPM(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSys.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted; change to finish when on-target if desired
    }
    
}
