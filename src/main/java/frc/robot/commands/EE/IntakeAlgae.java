package frc.robot.commands.EE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgae extends Command {
    private final Timer timer = new Timer();

    public IntakeAlgae () {
        addRequirements(EndEffector.getInstance());
        timer.reset();
        timer.start();
    }

    public void execute () {
        EndEffector.getInstance().setSpeed(Constants.EndEffector.INTAKE_ALGAE_SPEED);
    }

    public boolean isFinished () {
        return EndEffector.getInstance().isFrontTriggered()  || (timer.get() > 3);
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setSpeed(Constants.EndEffector.ALGAE_HOLD_SPEED);
    }
}