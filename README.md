MAE 155B Aircraft Design Project

Overview

This repository contains the MATLAB code for our MAE 155B aircraft design and analysis project.

This project is set up for teammates using Windows and PowerShell.

GitHub is the source of truth for the code. Do not use MATLAB Drive for editing the project.

⸻

First-Time Setup on Windows

1. Get access to the repository

Before doing anything else, make sure you have been added as a collaborator on GitHub.

⸻

2. Install Git for Windows

Install Git for Windows from the official Git website.

After installing, open PowerShell and verify Git is installed:

git –version

If Git is installed correctly, PowerShell will print a version number.

⸻

3. Create an SSH key

In PowerShell, run:

mkdir $HOME.ssh
ssh-keygen -t ed25519 -C “your_email@example.com”

Press Enter for all prompts.

This creates your SSH key.

⸻

4. Add your SSH key to GitHub

In PowerShell, run:

type $HOME.ssh\id_ed25519.pub

Copy the full output.

Then go to:

GitHub → Settings → SSH and GPG keys → New SSH key

Paste the copied key and save it.

⸻

5. Test your SSH connection

In PowerShell, run:

ssh -T git@github.com

If prompted, type:

yes

If everything is set up correctly, GitHub should respond with a message starting with:

Hi YOUR_USERNAME!

⸻

6. Clone the repository

In PowerShell, run:

cd $HOME\Desktop
git clone git@github.com:Darlock7/MAE155B-Aircraft-Design.git
cd MAE155B-Aircraft-Design

This will create a local copy of the repository on your Desktop.

⸻

Running the Project

1. Open MATLAB

Open MATLAB and set the Current Folder to:

C:\Users\YourUserName\Desktop\MAE155B-Aircraft-Design

Or, from PowerShell, you can open the folder first and then open MATLAB manually:

cd $HOME\Desktop\MAE155B-Aircraft-Design

⸻

2. Run the project

In MATLAB, run:

run_project
main

Always run run_project before main.

⸻

Daily Workflow

Before starting work

Open PowerShell and run:

cd $HOME\Desktop\MAE155B-Aircraft-Design
git pull

Then open MATLAB and run:

run_project
main

⸻

After making changes

Save your work in MATLAB.

Then in PowerShell, run:

cd $HOME\Desktop\MAE155B-Aircraft-Design
git add .
git commit -m “describe your changes”
git push

Use a short, clear commit message.

Examples:

git commit -m “Updated propulsion analysis”
git commit -m “Fixed airfoil path issue”
git commit -m “Improved mission profile calculations”

⸻

Project Structure

main.m
Main entry point for the project

run_project.m
Loads all project paths

src/geometry
Geometry design functions

src/aerodynamics
Aerodynamic analysis and supporting models

src/propulsion
Propulsion analysis functions

src/mission
Mission profile functions

src/energy
Energy calculation functions

src/economics
Optimization and cost analysis functions

plotting
Plotting and visualization utilities

data/airfoils
Airfoil .dat files

data/propellers
Propeller data files

data/models
Model and surrogate data files

data/reference
Reference text files

external/xfoil/bin
XFOIL executables used by the code

archive/old_main_versions
Older versions of main scripts kept for reference

⸻

Important Rules

Always run:

run_project

before running:

main

Always run:

git pull

before starting work.

Do not use MATLAB Drive as the source of truth.

Do not move, rename, or delete files unless you know exactly what you are doing.

Do not commit generated files such as:

xfoil_input_*
xfoil_polar_*
*.asv
conflict_copy

Do not edit directly on GitHub unless absolutely necessary.

⸻

Common PowerShell Commands

Go to the repo folder:

cd $HOME\Desktop\MAE155B-Aircraft-Design

Check Git status:

git status

Pull latest changes:

git pull

Stage all changes:

git add .

Commit changes:

git commit -m “your message”

Push changes:

git push

Show your SSH public key:

type $HOME.ssh\id_ed25519.pub

Test GitHub SSH connection:

ssh -T git@github.com

⸻

Common Problems

Git is not recognized

If PowerShell says git is not recognized, Git is not installed correctly. Install Git for Windows and restart PowerShell.

Check again with:

git –version

⸻

Permission denied (publickey)

If you see:

Permission denied (publickey)

your SSH key was not added correctly to GitHub.

Check your key with:

type $HOME.ssh\id_ed25519.pub

Then make sure that exact key is added to your GitHub account.

⸻

MATLAB cannot find files

Make sure you opened the correct repository folder and ran:

run_project

before running:

main

⸻

Standard Team Workflow
	1.	Open PowerShell
	2.	Go to the repo folder
	3.	Run git pull
	4.	Open MATLAB
	5.	Run run_project
	6.	Run main
	7.	Make your changes
	8.	Save your files
	9.	In PowerShell, run git add .
	10.	Run git commit -m "message"
	11.	Run git push
