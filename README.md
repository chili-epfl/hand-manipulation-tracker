# hand-manipulation-tracker
---------
libs used
---------

bloblib: http://opencvblobslib.github.io/opencvblobslib/  
opencv:  
        CV_MAJOR_VERSION 3  
        CV_MINOR_VERSION 1  

-----
Usage
-----

Use this code to determine which hand is controlling the cellulo robot.  
Values can be tweeked before or during the experiment, these values include:  
>>> lowH, highH: lower and higher bound for the Hue to be accepted in the skin detection process  
>>> top_cut, bot_cut: amount of unnecessary horizontal lines that should be ignored from the frame  
>>> lowV: lower bound for the white value to detect in the robot detection  
>>> LEN: assumed distance between the center of the hand and the robot, if the distance exceeds this value the blob is considered invalid  
>>> TRACE: increase to display more previous positions in the Kalman Filter  
>>> SENSITIVITY: increase to accept more irregularities in the movement of the hands  
                  when two hands touch each other, they will form a blob hence the centers will move drastically  
                  and a second blob will probably be detected on the edge of the frame  
                  this value will signal this blob merge by stamping a red circle on the screen  
>>> AREA: assumed area of the blob representing a hand  
>>> FLIP: change if you need to flip the frames you are receiving  
            set to 0 if no flips are needed,  
            1 for y axis,  
            2 for x axis,  
            3 for both  

--------------
Output Windows
--------------
>>> My Window: Contains the trackbars to tweek the values  
>>> kalman: measured and predicted positions of the hands  
>>> Blobs Image: detected blobs:  
                  green: right hand (on the screen)  
                  blue: left hand (on the screen)  
                  red: robot  
>>> frame: captured Image  
>>> test: skin detection filter  
>>> white: cellulo robot detection filter  

-------
Results
-------
Results can be found in two files:  
>>> logs.csv: position of the each hand, and whether or not it is controlling the robot  
                holds is 1 if it is controlling the robot  
                confirmed is 1 if the values in the line are likely to be correct  
>>> stats.txt: frequency of errors that happened and were detected by the Kalman filter  
                percentage of confirmed calculations  
