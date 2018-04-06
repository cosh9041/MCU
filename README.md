# MCU
Houses controller code with fault injection and fault management libraries for Team LLAMAS ASEN 4018/28 Senior Project 

### How To Get This Working on Your Machine:
1) Clone this repository
2) Once you've got it locally, open up `main_Script.ino` in the Arduino IDE. Download the Arduino IDE if you don't have it yet.
3) Update path to libraries folder by following these steps in the Arduino IDE:
    1) Go to File -> Preferences. 
    2) Update the sketchbook location with `path\to\MCU\repo`, where `path\to\MCU\repo` is replaced by whereever your cloned the MCU repo into your path. For example, this is my path:
![pathtomcurepo](https://user-images.githubusercontent.com/11947650/38004757-f77d559c-31fa-11e8-8699-401078a6b44c.PNG)
4) Make sure you're compiling using the Arduino DUE board. You can check this by going to Tools -> Board: {board_name_here}. Verify that {board_name_here} is some variation of Arduino DUE. If it's not available in the list of boards that comes up when you click on Board: {board_name_here}, click on Boards Manager... and download the DUE board interface software.

You should now be able to compile the sketch in Arduino without issues.
