package ca.mcgill.ecse211.lab5;

import java.util.ArrayList;

import org.apache.commons.math3.distribution.*;
import org.apache.commons.math3.exception.MathArithmeticException;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.TextLCD;

public class LightPollerColor {

	private SampleProvider lt;
	private float[] ltdata;
	public MotorControl motorcontrol;

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	 private Color red_block = new Color(0.20767974f,	0.027133795f,	0.015695887f,
	0.03038505f,	0.004516072f,	0.002923487f,
 "red");
	private Color blue_block = new Color(0.035873442f, 0.058556151f, 0.059581107f, 0.004077874f, 0.005821204f,
			0.004725411f, "blue");
	private Color yellow_block = new Color(0.30775123f, 0.221819854f, 0.246875f, 0.9973893f, 0.5888729f, 0.01106323f,
			"yellow");
	private Color white_block = new Color(0.325635933f, 0.246581879f, 0.145455755f, 0.032806904f, 0.033583433f,
			0.01739245f, "white");
	// private Color orange = new Color(0.164809809f, 0.128121575f, 0.096505883f,
	// 0.1574731f, 0.1548986f, 0.15302903f,
	// "orange");
	// private Color green = new Color(0.139549027f, 0.160666671f, 0.050921569f,
	// 0.19885207f, 0.18045974f, 0.16318f,
	// "green");
	// private Color table = new Color(0.139549027f, 0.160666671f, 0.050921569f,
	// 0.19885207f, 0.18045974f, 0.16318f,
	// "table");
	ArrayList<Color> colors = new ArrayList<Color>();

	public LightPollerColor(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
	}

	public void calibrate() {
		lt.fetchSample(ltdata, 0);

		for (int i = 0; i < 2; i++) {

			lt.fetchSample(ltdata, 0);
			System.out.println(ltdata[0] + "\t" + ltdata[1] + "\t" + ltdata[2]);
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		Sound.beep();
	}

	private float[] getAverageMeasurement() {
		float[] avg_values = { 0, 0, 0 };
		float red_sum = 0, green_sum = 0, blue_sum = 0;

		for (int i = 0; i < 10; i++) {
			lt.fetchSample(ltdata, 0);
			red_sum += ltdata[0];
			green_sum += ltdata[1];
			blue_sum += ltdata[2];
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		avg_values[0] = red_sum / 10;
		avg_values[1] = green_sum / 10;
		avg_values[2] = blue_sum / 10;
		return avg_values;
	}

	private double getProbability(Color color, float[] reading) {
		float sum_prob = 0, red_dist = 0, green_dist = 0, blue_dist = 0;
		float red_prob = 0, blue_prob = 0, green_prob = 0;

		red_dist = Math.abs(color.red_mean - reading[0]);
		green_dist = Math.abs(color.green_mean - reading[1]);
		blue_dist = Math.abs(color.blue_mean - reading[2]);

		red_prob = (float) (1 - color.red.probability(color.getRed_mean() - red_dist, color.getRed_mean() + red_dist));
		green_prob = (float) (1
				- (color.green.probability(color.getGreen_mean() - green_dist, color.getGreen_mean() + green_dist)));
		blue_prob = (float) (1
				- (color.red.probability(color.getBlue_mean() - blue_dist, color.getBlue_mean() + blue_dist)));

		System.out.println("red prob: " + red_prob);
		System.out.println("green prob: " + green_prob);
		System.out.println("blue prob: " + blue_prob);

		lcd.drawString("Red: " + Float.toString(red_prob), 0, 0);
		lcd.drawString("Green: " + Float.toString(green_prob), 0, 1);
		lcd.drawString("Blue: " + Float.toString(blue_prob), 0, 2);

		// returns the average probability of each probability
		return (red_prob + green_prob + blue_prob) / 3;
	}

	public Color max_color() {
		return yellow_block;
	}

	public void detectColor() {
		double prob_red = 0, prob_green = 0, prob_blue = 0, prob_orange = 10, prob_table = 0, prob_yellow = 0,
				prob_white = 0;
		double max_prob;
		lt.fetchSample(ltdata, 0);
		float[] reading = getAverageMeasurement();

		System.out.println("red read: " + reading[0]);
		System.out.println("green read: " + reading[1]);
		System.out.println("blue read: " + reading[2]);

		// prob_red = getProbability(red_block, reading);
		// prob_green = getProbability(blue_block, reading);
		// prob_blue = getProbability(yellow_block, reading);
		prob_yellow = getProbability(yellow_block, reading);
		prob_blue = getProbability(blue_block, reading);
		prob_red = getProbability(red_block, reading);
		prob_white = getProbability(white_block, reading);

		yellow_block.setProbability(prob_yellow);
		blue_block.setProbability(prob_blue);
		red_block.setProbability(prob_red);
		white_block.setProbability(prob_white);

		System.out.println("Orange" + prob_orange);
		System.out.println("Green" + prob_green);
		System.out.println("Table" + prob_table);
		System.out.println("YELLOW" + prob_yellow);

		max_prob = Math.max(Math.max(Math.max(prob_blue, prob_green), prob_red), prob_white);

		// lcd.drawString("Red: " + Float.toString(reading[0]), 0, 3);
		// lcd.drawString("Green: " + Float.toString(reading[1]), 0, 4);
		// lcd.drawString("Blue: " + Float.toString(reading[2]), 0, 5);
		// lcd.drawString("Orange: " +Double.toString(prob_orange), 0, 3);

		if (max_prob < 0.75) {
			System.out.println("unknown color");
		}

		else if (max_prob == prob_red) {
			System.out.println("its Red with a probability of " + prob_red);
		} else if (max_prob == prob_blue) {
			System.out.println("its blue with a probability of " + prob_blue);
		} else if (max_prob == prob_yellow) {
			System.out.println("its yellow with a probability of " + prob_yellow);
		} else {
			System.out.println("its white with a probability of " + prob_white);
		}
	}

	public boolean target_found(Color target) {
		lt.fetchSample(ltdata, 0);
		float[] reading = getAverageMeasurement();

		red_block.setProbability(getProbability(red_block, reading));
		blue_block.setProbability(getProbability(blue_block, reading));
		yellow_block.setProbability(getProbability(yellow_block, reading));
		white_block.setProbability(getProbability(yellow_block, reading));

		colors.add(red_block);
		colors.add(blue_block);
		colors.add(yellow_block);
		colors.add(white_block);

		if (max_color(colors).name == target.name) {
			Sound.beep();
			Sound.beep();
			colors.clear();
			return true;
		} else {
			Sound.beep();
			colors.clear();
			return false;
		}

	}

	public Color max_color(ArrayList<Color> list) {
		Color max_color = list.get(0);
		for (int i = 0; i < 4; i++) {
			if (max_color.getProbability() < list.get(i).getProbability()) {
				max_color = list.get(i);
			}
		}
		return max_color;
	}

}
