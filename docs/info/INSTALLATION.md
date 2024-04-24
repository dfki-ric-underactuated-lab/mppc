# Software Installation Guide

This guide is written for the Linux operating system using Ubuntu.

## Dependencies #
To guarantee long-term reliable installation a specific Python version 3.10 is required. This version can be installed by first running:
```
sudo apt-get update
```
and then:
```
sudo apt-get install python3.10
```
The tool `poetry` is used for dependency management and packaging in Python. As of 15.04.2024 installing it (on Linux, Ubuntu 22.04) is as simple as running:
```
curl -sSL https://install.python-poetry.org | python3 -
```
You may have to install `curl` first if you don't have it with:
```
sudo apt-get update && sudo apt install curl
```

## Cloning the Repository #
Go into your parent folder for GitHub projects and then command:
```
git clone git@github.com:dfki-ric-underactuated-lab/mppc.git
```

## Installation #
Move into the parent folder of the repository with:
```
cd mppc
```
If you have not already extended your python path to one of the parent folders of this repository, you should do so with:
```
echo "export PYTHONPATH=$(pwd):\$PYTHONPATH" >> ~/.bashrc && . ~/.bashrc
```
A virtual environment is then created in the parent folder of the repository with: 
```
poetry shell
```
Dependencies of the software are listed [here](../../pyproject.toml) and are installed with:
```
poetry install
```

## Running Code #

To run any code you need to be inside the repository parent folder. If you haven't already, activate the virtual environment using poetry by running the command:
```
poetry shell
```
To see the robot completing a parkour in simulation, you can command:
```
python software/experiment/experiment_mppc.py 1
```
If you want to see a replay of an actual experiment in the real world with controller insights, you can command:
```
python software/visualization/video/replay.py <experiment>
```
Replace `<experiment>` with a number corresponding to the experiment you want to see:

| `<experiment>` | label        |
| :------------: | ------------ |
|       1        | static       |
|       2        | dynamic_1    |
|       3        | dynamic_2    |
|       4        | dynamic_3    |
|       5        | dynamic_4    |
|       6        | two_rounds   |
|       7        | disturbances |
