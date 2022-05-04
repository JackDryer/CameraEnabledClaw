# CameraEnabledClaw
https://github.com/JackDryer/CameraEnabledClaw

To run: run the UI script

If the printer is connected to the correct hardware it will connect, otherwise it will use a virtual representation of the hardware 

To use:
You can switch the input via the drop down at the top
Fist of all, enter the configure hsvs menu and click on the screen to set the colour value of the selected pixel to the point selected in the menu.

Next the code needs to be calibrated. In the calibrate menu, click “use average” to get a more consistent reading across frames. When you are happy with a reading at one height, click “good read” and then change the height and select “calculate  y offset” to calculate the y offset, you may need to deselect then reselect “use average” to get an accurate reading.
There is no need to click “good read” again as the calibration will bee the last values that were displayed.

If the program has difficulty detecting the object, go into the “configure automatic mode” section, again you can select the colour by clicking on the screen. The best order is to to turn the closings, openings and inflations to 0, to pick a good colour, to refine the tolerance, then closings then openings, leaving inflations at 0.

The frame does not update when the printer is in motion
