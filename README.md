# InductionMotorControl
Controlling 3 Phase AC Asynchronous Induction Motor

Check [This](https://github.com/GhajariAli/DCMotorControl) Repo for DC motor Control

<details>
<summary> Preparation </summary>
  
  <details>
    
  <summary> Motor Nameplate </summary>
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/79e54aa8-2c5a-48e7-a765-f435c64f1b8a)
  
  </details>
  
  <details>
    
  <summary> Designing the circuit </summary>
  
  I've Designed the control boad around this IC fro Infineon [IGCM04F60GAXKMA1](https://www.digikey.ca/en/products/detail/infineon-technologies/IGCM04F60GAXKMA1/5960098)
  
  Here is the video of testing the circuit and getting initial result:
  
   [ ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/e5c6398d-9890-4f7f-9bff-9ceaebefc4f3)](https://www.youtube.com/shorts/fg0to3o2meA)
  
  So I designed a PCB to get things going Faster, you can find the KiCAD design and initial code to run the motor in [THIS](https://github.com/GhajariAli/InductionMotorControl/tree/FirstCurcuitDesign) branch
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/7d180d9a-082e-4a4d-8e66-88960f138076)
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/3f589aea-a336-4bb9-9014-de208c041ac9)
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/ef48dafa-2c1b-447d-9f68-5755eeecb492)
  
  </details>
  
  <details>
    
  <summary> Adding Encoder </summary>
  Induction Motors are usally not controlled by a closed loop using encoder but to validate and verify my own work I'd like to have and encoder on the shaft.
  
  [here](https://www.aliexpress.com/item/4000094589182.html?spm=a2g0o.order_list.order_list_main.56.1fdd1802ZbFUS8) is the encoder I've used : 
  
  [![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/795896b8-fc90-404c-ae55-26a9536d124e)](https://www.aliexpress.com/item/4000094589182.html?spm=a2g0o.order_list.order_list_main.51.55f218029jO34n)
  
  Below is what I've designed to mount the encoder, you'll find all 3D models in the same branch as mentioned in the circuit diagram section.
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/297e91bb-acf1-4c62-90d7-24dd36e4b6fe)
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/8133c074-e35b-4a94-ba1a-8fad14bfdc7c)
  
  Update: had to dessign and print the belt with TPU as it was not standard size turned out good:
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/2ccf072d-3609-425e-9b5c-a8dbe1ca8d47)![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/12f8af9a-5037-403b-a2b7-296931cb89d1)
  
  </details>
  
  
  <details>
    
  <summary> Testing the PCB </summary>
  PCB came very good. had to change couple resistors and design a base for it:
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/a908310f-9bc8-4ede-8330-97254afd50d5)
  
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/dc448794-1e92-4b78-886b-6795ac4cf47f)
  
  and IT WORKS THE FIRST TRY!
  </details>
</details>

<details>
<summary> SPWM speed Control </summary>
  
  ![image](https://github.com/GhajariAli/InductionMotorControl/assets/124516346/6f450de8-868a-4954-a6fa-8d99349a4dc9)

this made the motor run smoother, current is now lower and it makes less noise.
</details>
