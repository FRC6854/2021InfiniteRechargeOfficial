package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LEDController extends SubsystemBase implements RobotMap {
	public static final int num_leds = 60;

	public static enum LEDMode {
		LCF,
		LCB,
		UCF,
		UCB,
		BOTH_FWRD,
		BOTH_BKWD,
		NO_VISION,
		VISION,
		CLIMB_ACTIVE,
		WINCH_ACTIVE,
		LOW_VOLTAGE,
		DEFAULT
	};

	private static LEDController instance = null;
	private AddressableLED addrled;
	private AddressableLEDBuffer ledbuf;
	private Color alliance_colour;
	private LEDMode current_mode = LEDMode.DEFAULT;
	private int counter = 0;

	public static final int led_update_wait_cycle = 3;

	int state=0;

	public LEDController() {
		if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
			alliance_colour = new Color(0, 0, 255);
		} else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
			alliance_colour = new Color(255, 0, 0);
		} else {
			alliance_colour = new Color(0, 0, 0);
		}
		addrled = new AddressableLED(PWN_ADDRESSABLE_LED);
		ledbuf = new AddressableLEDBuffer(num_leds);
		addrled.setLength(ledbuf.getLength());
		addrled.setData(ledbuf);
		addrled.start();
		set_default();
	}

	// Default: full alliance colour
	private void set_default() {
		System.out.println("DEFAULT");
		for (int i = 0; i < num_leds; i++) {
			ledbuf.setLED(i, alliance_colour);
		}
		addrled.setData(ledbuf);
	}

	// Winch active, set all Green
	private void set_winch_active() {
		System.out.println("WINCH");
		for (int i = 0; i < num_leds; i++) {
			ledbuf.setRGB(i, 0, 255, 0);
		}
		addrled.setData(ledbuf);
	}

	// foward pattern, alliance colour
	private void set_both_forward() {
		System.out.println("FORWARD");
		for (int i = 0; i < num_leds; i++) {
			if (i == counter) {
				ledbuf.setLED(i, alliance_colour);
			} else {
				ledbuf.setRGB(i, 0, 0, 0);
			}
		}
		addrled.setData(ledbuf);
		counter++;
		if (counter == num_leds) {
			counter = 0;
		}
	}

	// backward pattern, alliance colour
	private void set_both_backward() {
		System.out.println("BACKWARD");
		for (int i = 0; i < num_leds; i++) {
			if (i == counter) {
				ledbuf.setLED(i, alliance_colour);
			} else {
				ledbuf.setRGB(i, 0, 0, 0);
			}
		}
		addrled.setData(ledbuf);
		counter--;
		if (counter < 0) {
			counter = num_leds - 1;
		}
	}

	@Override
	public void periodic() {
		if(state == 0){
			switch (current_mode) {
				case DEFAULT:
					set_default();
					break;
				case WINCH_ACTIVE:
					set_winch_active();
					break;
				case BOTH_FWRD:
					set_both_forward();
					break;
				case BOTH_BKWD:
					set_both_backward();
					break;
				default:
					System.out.println(current_mode);
					break;
			}
		}
		state++;
		if(state == led_update_wait_cycle){
			state = 0;
		}
	}

	public void setMode(LEDMode mode) {
		/// the setMode(DEFAULT) get called randomly, so do not set the mode to DEFAULT
		if (mode != LEDMode.DEFAULT) {
			current_mode = mode;
		}
	}

	public static LEDController getInstance() {
		if (instance == null) instance = new LEDController();
		return instance;
	}
}
