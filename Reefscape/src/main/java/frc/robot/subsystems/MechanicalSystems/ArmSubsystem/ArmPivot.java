package frc.robot.subsystems.MechanicalSystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivot extends SubsystemBase {

    private ArmSubsystem armSubsystem;  // Reference to ArmSubsystem

    public ArmPivot(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;  // Set the reference to ArmSubsystem
    }

    // Method to get wrist position
    public double getPivotPosition(){
        return 0.0;  // Placeholder for wrist position
    }

    @Override
    public void periodic() {
    }
}


//See ArmWrist for updates