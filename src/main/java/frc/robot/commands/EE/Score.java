package frc.robot.commands.EE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.EndEffector;

public class Score extends Command {
    private final Timer timer = new Timer();

    public Score () {
        addRequirements(EndEffector.getInstance());
        timer.reset();
        timer.start();
    }

    public void execute () {
        EndEffector.getInstance().setSpeed(RobotMap.EndEffector.OUTTAKE_SPEED);
    }

    public boolean isFinished () {
        return !EndEffector.getInstance().hasAlgae() && !EndEffector.getInstance().hasCoral() && (timer.get() > 10);
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setSpeed(0);
    }
}
