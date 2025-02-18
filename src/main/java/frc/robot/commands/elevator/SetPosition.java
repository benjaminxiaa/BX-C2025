package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetPosition extends InstantCommand {

    private double height;

    public SetPosition(double height) {
        addRequirements(Elevator.getInstance());
        this.height = height;
    }

    public void execute() {
        Elevator.getInstance().setDesiredPosition(height);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}