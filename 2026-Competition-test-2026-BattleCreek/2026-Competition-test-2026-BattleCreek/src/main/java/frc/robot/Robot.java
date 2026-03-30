package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private Command autonomousCommand;


    @Override
    public void robotInit() {
        // If publishing to NetworkTables and DataLog
        DataLogManager.start();
        URCL.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        robotContainer.updateInterface();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if(autonomousCommand != null) autonomousCommand.cancel();
    }

}
