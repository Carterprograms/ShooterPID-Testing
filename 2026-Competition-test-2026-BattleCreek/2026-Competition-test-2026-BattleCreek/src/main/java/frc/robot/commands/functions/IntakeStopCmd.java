package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;

public class IntakeStopCmd extends Command {
    
    private final IntakeSys intakeSys;

    public IntakeStopCmd(IntakeSys intakeSys) {
        this.intakeSys = intakeSys;
    }

    @Override
    public void execute() {
        intakeSys.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
