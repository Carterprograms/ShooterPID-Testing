package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.subsystems.AgitatorSys;

public class AgitatorCmd extends Command {

    private final AgitatorSys agitatorSys;
    private final boolean reverse;

    public AgitatorCmd(AgitatorSys agitatorSys, boolean reverse) {
        this.agitatorSys = agitatorSys;
        this.reverse = reverse;
    }

    @Override
    public void execute() {
        if(reverse == true){
            agitatorSys.setAgitatorRPM(AgitatorConstants.agitatorRPM, true);
        }else{
            agitatorSys.setAgitatorRPM(AgitatorConstants.agitatorRPM, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        agitatorSys.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
