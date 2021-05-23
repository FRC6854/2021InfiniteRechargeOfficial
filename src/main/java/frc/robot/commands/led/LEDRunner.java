package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDController;

public class LEDRunner extends CommandBase {
	private LEDController ledctrl = LEDController.getInstance();
	public static final int led_update_wait_cycle = 3;

	int state=0;

	public LEDRunner(){
		addRequirements(ledctrl);
	}

	@Override
	public void execute(){
		if(state == 0){
			ledctrl.update();
		}
		state++;
		if(state == led_update_wait_cycle){
			state = 0;
		}
	}

}
