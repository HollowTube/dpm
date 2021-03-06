package ca.mcgill.ecse211.main_package;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta) and also show what task the robot is performing at all times.
 * This allows for easier understanding of the problems encountered
 * when testing for hardware and software tasks.
 * The display could also be programmed to display other values for testing purposes.
 * The display will run as a separate thread. 
 * <p>
 * It will not be used during the beta demonstration or the final presentation.
 * 
 * @author Tritin
 * 
 */
public class Display implements Runnable {

	private Odometer odo;
	private TextLCD lcd;
	private double[] position;
	private final long DISPLAY_PERIOD = 25;
	private long timeout = Long.MAX_VALUE;

	/**
	 * This is the class constructor which is comprised of any text that is to be displayed.
	 * 
	 * @param lcd text to display
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;
	}

	/**
	 * This is the overloaded class constructor which is comprised of any text that is to be displayed
	 * and a timeout pause of the thread.
	 * 
	 * @param lcd text to display
	 * @param timeout Pause time
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.timeout = timeout;
		this.lcd = lcd;
	}

	/**
	 * Method to run the display thread.
	 */
	public void run() {

		lcd.clear();

		long updateStart, updateEnd;

		long tStart = System.currentTimeMillis();
		do {
			updateStart = System.currentTimeMillis();
			// Retrieve x, y and Theta information
			position = odo.getXYT();
			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
			lcd.drawString("State " + Main.state.name(), 0, 3);
			
			// this ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} while ((updateEnd - tStart) <= timeout);

	}

}
