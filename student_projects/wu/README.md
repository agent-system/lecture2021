# Garbage Collection using Salamander and Tiago Titanium

## 1.Introduction
Three .wbt files are available in this project:

a. **gc_sala.wbt** is the file for garbage collection using salamander, the controller is internal and is written in C.  
b. **gc_sala_irteusgl.wbt** is also the file for garbage collection using salamander, but the controller is set to be external and is written in euslisp.  
c. **gc_tiago_irteusgl.wbt** is the file for garbage colletion using Tiago Titanium, the controller is set to be external and is written in euslisp.  

## 2.Environment setup
This project is built based on **Webots R2021a**.  

To use euslisp, please install euslisp following https://github.com/euslisp/EusLisp.  

controllers/irteusgl/tiago_interface.l is the Tiago Titanium model file for euslisp.   

controllers/irteusgl/webotslib.l is the file allowing euslisp to call webots functions.  

## 3.How to run
a. **gc_sala.wbt**  
    `cd student_projects/wu`  
    `webots worlds/gc_sala.wbt`  
    `//build the controller in webots and start the simulation`  

b. **gc_sala_irteusgl.wbt**  
    `cd student_projects/wu`  
    `webots worlds/gc_sala_irteusgl.wbt`  
    `//in another terminal`  
    `irteusgl controllers/irteusgl/salamander.l`  

c. **gc_tiago_irteusgl.wbt**  
    `cd student_projects/wu`  
    `webots worlds/gc_tiago_irteusgl.wbt`  
    `//in another terminal`  
    `irteusgl controllers/irteusgl/tiago.l`  

## 4.Problems left  
a. When executing salamander.l, as the state of the garbage(box) on the pool-side is unstable, the simulation may be dumped. Also, in some cases, the garbage(box) starts spinning even without touching any objects, which can also lead to irregular conditions.  

b. The orientation information of the garbage is currently not utilized for garbage picking using Tiago Titanium, so, changing the orientation of the garbage in gc_tiago_irteusgl.wbt can cause failure in picking up the garbage.  

c. When throwing the garbage, the arm of Tiago Titanium is simply controlled with forward-kinematics, without using the position information of the garbage bin. So, changing the position or oriention of the garbage bin in gc_tiago_irteusgl.wbt can lead the Tiago Titanium to throw garbage out of the garbage bin.

