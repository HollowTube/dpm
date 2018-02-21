package ca.mcgill.ecse211;

import lejos.hardware.sensor.*;

public class UltrasonicLocalizer {
	private static int wall_distance=30;
	private static int noise_margin=2;
	private final Odometer odo;
	private final Navigation nav;
	private final EV3UltrasonicSensor us;
	private final int type;
	public float[] usData;
	public double[] xyt;
	public double[] edgeDetect;
	private int dTheta;
	
	public UltrasonicLocalizer(Odometer odo, Navigation nav, EV3UltrasonicSensor us, int type){
		this.odo=odo;
		this.nav=nav;
		this.us=us;
		this.type=type;
		this.us.enable();
	}
	public void Localize(){
		if(this.type==1){
			fallingEdge();
		}else if(this.type==2){
			risingEdge();
		}
		if(edgeDetect[0]<edgeDetect[1]){
			double correctionAngle = 225-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
		}
		else{
			double correctionAngle = 45-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
		}
	}
	public void fallingEdge(){
		while(getDistance()>(wall_distance+noise_margin)){
			nav.turnTo(dTheta);
		}
		xyt = odo.getXYT();
		edgeDetect[0] = xyt[2];
		while(getDistance()<(wall_distance-noise_margin)){
			nav.turnTo(dTheta);
		}
		xyt = odo.getXYT();
		edgeDetect[1] = xyt[2];
	}
	public void risingEdge(){
		while(getDistance()<(wall_distance-noise_margin)){
			xyt = odo.getXYT();
			edgeDetect[0] = xyt[2];
		}
		while(getDistance()>(wall_distance+noise_margin)){
			xyt = odo.getXYT();
			edgeDetect[1] = xyt[2];
		}
		
	}
	public float getDistance(){
		us.fetchSample(usData, 0);
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}
		if(usData[0]>255){
			usData[0]=255;
		}
		return usData[0];
	}
}
