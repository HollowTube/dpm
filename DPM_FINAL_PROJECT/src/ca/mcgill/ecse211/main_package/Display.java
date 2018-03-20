package ca.mcgill.ecse211.main_package;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta).
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
	 * This is the class constructor.
	 * 
	 * @param odoData
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;
	}

	/**
	 * This is the overloaded class constructor.
	 * 
	 * @param odoData
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
			// dist = uspoller.getDist();
			// Retrieve x, y and Theta information
			position = odo.getXYT();
			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
			lcd.drawString("State " + Main.state.name(), 0, 3);

			// lcd.drawString("New Heading: " +
			// numberFormat.format(Navigation.final_heading), 0, 3);
			// lcd.drawString("Distance: " + numberFormat.format(dist), 0, 4);
			// lcd.drawString("dx: " + numberFormat.format(Odometer.deltaL), 0, 5);
			// lcd.drawString("dy: " + numberFormat.format(Odometer.deltaR), 0, 6);
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
