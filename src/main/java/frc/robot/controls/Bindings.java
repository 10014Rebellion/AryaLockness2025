package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Bindings {
        private final ActionCommands mActionCmd;
        private final CommandXboxController mController = new CommandXboxController(0);

        
        
        public Bindings(){

            mActionCmd = new ActionCommands();
                
        }

        public void initBindings(){
            mController.rightBumper().whileTrue(mActionCmd.getIntakeCoralCmd());
            mController.y().whileTrue(mActionCmd.prepCoralL1());
            mController.x().whileTrue(mActionCmd.scoreCoralL1());
        }
    
        
    
}
