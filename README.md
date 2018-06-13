
# Traffic Light Controller
Embedded traffic light controller created for Digital Systems Design, CSU Chico Spring 2018.
![Figure 1](https://i.imgur.com/uawQQfq.png)
There are four inputs: 1) A traffic sensor on 1st St, 2) a traffic sensor on Warner St, 3) A walk request button on 1st St, and 4) a walk request button on Warner St. There are 8 outputs: 2 traffic lights each with red, yellow, and green, and 2 walk signals, 1 across each street.

## Behavior

 - If Traffic is coming through Warner St. and no traffic in 1st st. (SW1=1, SW2=0) then S1=Green, S2=Red. 
 - If Traffic is coming through 1stSt and no traffic in Warner st. (SW1=0, SW2=1), S1=Red, S2=Green.
 - No traffic on any road (SW1=0, SW2=0), gives priority to Warner st.
 - To change from Green to Red implement a yellow light for 2seconds (wait state for each of the two lights)
 - Green lights should last for 7sec.
 - If cars are coming in all directions (SW1=1, SW2=1), cycle through all the states.
 - If walk request is initiated on Warner street (B1=1, B2=0), traffic on Warner street and 1ststreet should be stopped (S1=Red, S2=Red) and the walk signal for Warner streets should go ON for 4 sec.
 - If walk request is initiated on 1ststreet only (B1=0, B2=1), then S1=Green, S2=Red and the walk signal for 1ststreet should go ON for 4 sec.
 - If no walk requests were received, the walk signal should be OFF on both streets.

## Completed system
![Image 1](https://i.imgur.com/XnPeldi.jpg)
[Video of working system (youtube)](https://www.youtube.com/watch?v=hmjl9-jLBSY)
