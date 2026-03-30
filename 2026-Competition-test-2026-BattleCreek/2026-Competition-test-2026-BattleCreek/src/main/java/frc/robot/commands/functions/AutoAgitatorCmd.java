package frc.robot.commands.functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.subsystems.AgitatorSys;

public class AutoAgitatorCmd extends Command {

    private final AgitatorSys agitatorSys;
    private final Timer timer;
    private final double duration; // seconds
    private static final double START_DELAY = 0.5; // seconds
    boolean started;

    public AutoAgitatorCmd(AgitatorSys agitatorSys, double duration) {
        this.agitatorSys = agitatorSys;
        this.duration = duration;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        started = false;
    }

    @Override
    public void execute() {
        // Wait START_DELAY seconds before starting the agitator
        if (timer.hasElapsed(START_DELAY)) {
            agitatorSys.setAgitatorRPM(AgitatorConstants.agitatorRPM, false);
            started = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        agitatorSys.stop();
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Finish after the initial delay plus the intended duration
        return timer.hasElapsed(START_DELAY + duration);
    }
    
}