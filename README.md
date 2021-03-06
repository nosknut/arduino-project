## Setup
- Install VSCode
    - https://code.visualstudio.com/download

- Install Git
    - https://git-scm.com/download/win
    - Configure git user in the terminal
        ```
        git config --global user.name "My Name"
        git config --global user.email "myemail@example.com"
        ```

- Clone the project with VSCode
    - https://github.com/nosknut/arduino-project.git

- Install python
    - [Download](https://www.python.org/downloads/)
    - [Tutorial](https://realpython.com/installing-python/)
- Install [PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
    - [Tutorial for setup and use](https://youtu.be/JmvMvIphMnY)

- Setup workspace
    - After you have created several PlatformIO projects, run the [find-workspaces.py](./find-workspaces.py) file with the following command:
        ```python ./find-workspaces.py```
        This will generate VSCode workspaces. This will let you work on multiple
        projects in a single git repository without opening vscode in each respective project folder separately. You have to run this script every time you add a new project.

<br />

## Unit Testing
- Install GCC
    - https://www.youtube.com/watch?v=8Ib7nwc33uA&ab_channel=DerekBanas
- Select Environment to "native"
- "Start Debugging" through the PlatformIO extension

<br />

## Deprecated
### Setup VSCode Arduino extension
- Install ArduinoIDE
    - https://www.arduino.cc/en/software


- Install the VSCode Arduino Extension
    - https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino

- After creating a project and main.ino file
    - ctrl + shift + P
    - Type: Arduino: Initialize
    - Hit Enter
    - Select the main.ino file
    - Select the correct board
    - After some time the IDE should generate all required configs
