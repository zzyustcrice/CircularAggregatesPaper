# Circular Aggregates Paper Code
Uses Maven based project infrastructure

Source code is in Java language

## Build instructions

- Use an IDE (e.g. Intellij Idea or Netbeans) and setup a maven based project
- (or) in command window run: 
<code>mvn package</code>
- Maven compiles and packages simulation code to <code>CAPaperCode-1.0-SNAPSHOT.jar</code> and all the required dependencies are copied to <code>lib/</code>. Compiled packages are available in <code>target/</code> directory.

## Run instructions

 Copy following files and folders to a separate folder for simulation run:
  * <code>CAPaperCode-1.0-SNAPSHOT.jar</code>
  * <code>lib/</code>
  * <code>Flexicell19_Parameters.xml</code>
  * <code>GlobalParameters_NonGUI.txt</code>

Run simulation using the command in OS command window: <code>java -jar CAPaperCode-1.0-SNAPSHOT.jar</code>

## Simulation run files

 While simulation is running snapshots of current simulation status are stored in <code>images/</code> folder and simulation information is written to files in <code>runData/</code> folder. 

## Precompiled packages

 Precompiled jar packages are available in <code>packages/</code> folder of the github repository 