package ca.mcgill.ecse211;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class Boot_Menu {
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public int SC;		//ranges from [0,3]
	public int TB;		//ranges from [1,4]
	public int LLx;		//ranges from [0,8]
	public int LLy;
	public int URx;
	public int URy;
	public Boot_Menu(){}
	public void start(){
		int buttonChoice;
		int cursor = 1;;
		while(true){
			do {
				// clear the display
				lcd.clear();
				// ask the user whether the robot should anticipate obstacles
				lcd.drawString("   Select SC    ", 0, 0);
				lcd.drawString("SC 0", 0, 1);
				lcd.drawString("SC 1", 0, 2);
				lcd.drawString("SC 2", 0, 3);
				lcd.drawString("SC 3", 0, 4);
				lcd.drawString("<--", 8, cursor);
				buttonChoice = Button.waitForAnyPress(); // Record choice
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=4){
					cursor++; //make the cursor move down
				}else{
					cursor=4; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				SC=cursor-1;
				break;
			}
		}
		cursor = 1;
		while(true){	
			do {
				// clear the display
				lcd.clear();
				// ask the user whether the robot should anticipate obstacles
				lcd.drawString("   Select TB    ", 0, 0);
				lcd.drawString("TB 1", 0, 1);
				lcd.drawString("TB 2", 0, 2);
				lcd.drawString("TB 3", 0, 3);
				lcd.drawString("TB 4", 0, 4);
				lcd.drawString("<--", 8, cursor);
				buttonChoice = Button.waitForAnyPress(); // Record choice
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=4){
					cursor++; //make the cursor move down
				}else{
					cursor=4; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				TB=cursor;
				break;
			}
		}
		cursor = 1;
		while(true){	
			do {
				if(cursor<6){
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("   Select LLX    ", 0, 0);
					lcd.drawString("LLx 0", 0, 1);
					lcd.drawString("LLx 1", 0, 2);
					lcd.drawString("LLx 2", 0, 3);
					lcd.drawString("LLx 3", 0, 4);
					lcd.drawString("LLx 4", 0, 5);
					lcd.drawString("<--", 8, cursor);
				}else{
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("LLx 5", 0, 0);
					lcd.drawString("LLx 6", 0, 1);
					lcd.drawString("LLx 7", 0, 2);
					lcd.drawString("LLx 8", 0, 3);
					lcd.drawString("<--", 8, cursor-6);
				}
				buttonChoice = Button.waitForAnyPress(); // Record choice
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=9){
					cursor++; //make the cursor move down
				}else{
					cursor=9; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				LLx=cursor-1;
				break;
			}
		}
		cursor = 1;
		while(true){	
			do {
				if(cursor<6){
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("   Select LLY    ", 0, 0);
					lcd.drawString("LLy 0", 0, 1);
					lcd.drawString("LLy 1", 0, 2);
					lcd.drawString("LLy 2", 0, 3);
					lcd.drawString("LLy 3", 0, 4);
					lcd.drawString("LLy 4", 0, 5);
					lcd.drawString("<--", 8, cursor);
				}else{
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("LLy 5", 0, 0);
					lcd.drawString("LLy 6", 0, 1);
					lcd.drawString("LLy 7", 0, 2);
					lcd.drawString("LLy 8", 0, 3);
					lcd.drawString("<--", 8, cursor-6);
				}
				buttonChoice = Button.waitForAnyPress(); // Record choice
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=9){
					cursor++; //make the cursor move down
				}else{
					cursor=9; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				LLy=cursor-1;
				break;
			}
		}
		cursor = 1;
		while(true){	
			do {
				if(cursor<6){
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("   Select URX    ", 0, 0);
					lcd.drawString("URx 0", 0, 1);
					lcd.drawString("URx 1", 0, 2);
					lcd.drawString("URx 2", 0, 3);
					lcd.drawString("URx 3", 0, 4);
					lcd.drawString("URx 4", 0, 5);
					lcd.drawString("<--", 8, cursor);
				}else{
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("URx 5", 0, 0);
					lcd.drawString("URx 6", 0, 1);
					lcd.drawString("URx 7", 0, 2);
					lcd.drawString("URx 8", 0, 3);
					lcd.drawString("<--", 8, cursor-6);
				}
				buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=9){
					cursor++; //make the cursor move down
				}else{
					cursor=9; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				URx=cursor-1;
				break;
			}
		}
		cursor = 1;
		while(true){	
			do {
				if(cursor<6){
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("   Select URY    ", 0, 0);
					lcd.drawString("URy 0", 0, 1);
					lcd.drawString("URy 1", 0, 2);
					lcd.drawString("URy 2", 0, 3);
					lcd.drawString("URy 3", 0, 4);
					lcd.drawString("URy 4", 0, 5);
					lcd.drawString("<--", 8, cursor);
				}else{
					// clear the display
					lcd.clear();
					// ask the user whether the robot should anticipate obstacles
					lcd.drawString("URy 5", 0, 0);
					lcd.drawString("URy 6", 0, 1);
					lcd.drawString("URy 7", 0, 2);
					lcd.drawString("URy 8", 0, 3);
					lcd.drawString("<--", 8, cursor-6);
				}
				buttonChoice = Button.waitForAnyPress(); // Record choice
			} while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice !=Button.ID_ENTER);
			if (buttonChoice == Button.ID_UP) {
				if(cursor!=1){
					cursor--; //make the cursor move up
				}else{
					cursor=1; //if at the top, keep it there
				}
			}
			else if (buttonChoice == Button.ID_DOWN) {
				if(cursor!=9){
					cursor++; //make the cursor move down
				}else{
					cursor=9; //if at the bottom, keep it there
				}
			}
			else if(buttonChoice == Button.ID_ENTER){
				URy=cursor-1;
				break;
			}
		}
	}
}
