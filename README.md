# FBIK_Project
The purpose of this project is 
1. Implementation of full-body inverse kinematics (FBIK) 
2. Explore general process of how to use analytic and numerical optimization
3. Get familiar with how to calculate Jacobian matrix
4. Get familiar with Unity Engine and C#

## 1. Requirments
### Code is written in C# and uses Unity Engine for framework 
This project requires:
* Unity
 * 2020.3.33f1
* Library
  * ALGLIB for C# (https://www.alglib.net/download.php#csharp) to use optimization modules

----------

## 2. Introduction
### Limitation of two-bone inverse kinematics 
Consider about the situation of  <br/>
  * Task : stretch hand to reach a red ball <br/>
  * End-effector: Left Hand <br/>
  * Target: Red ball

<image src = "https://user-images.githubusercontent.com/45995611/176603221-77e0fe51-6171-49df-ad6a-a9bf966f9bbe.png" width="40%" height="40%"></left>
<image src = "https://user-images.githubusercontent.com/45995611/176603199-5685462d-2eeb-484d-b7a9-98870eb7e9a9.png" width="40%" height="40%"></right>

----------

### Benefits of two-bone inverse kinematics 
Ability to transfer control from the top of the hierarchy to the bottom
* Natural poses and animations for characters <br/>
* Translate + rotate body parts with FBIK end-effectors <br/>
* Rest of body follow the movements <br/>

<image src = https://user-images.githubusercontent.com/45995611/176604350-52c8e581-3d29-454f-9c8d-1db19fd43644.png width="40%" height="40%"></center>

## 3. Result
<image src = https://user-images.githubusercontent.com/45995611/176606587-4cdd6456-ab3d-438f-88e5-45977779d8c4.gif>


## 4. Presentation PowerPoint
Please request me by email if needed (yrs1107@gmail.com) 
