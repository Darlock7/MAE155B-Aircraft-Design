**MAE 155B Aircraft Design Project Setup and Workflow (Windows)**
Overview

**First-Time Setup on Windows**

**1.	Get Repository Access**

Make sure you have been added as a collaborator on the GitHub repository before starting.

**2.Install Git for Windows**

Link: https://github.com/git-for-windows/git/releases/download/v2.54.0.windows.1/Git-2.54.0-64-bit.exe
Download and install Git. After installation, open PowerShell and verify Git is installed by running:

	git -–version

If installed correctly, it should return a version number.

⸻

**3.	Create an SSH Key**

	mkdir $HOME.ssh
	ssh-keygen -t ed25519 -C “your_email@example.com”

Press Enter through all prompts unless you want a passphrase.

This creates:
	•	Private key: id_ed25519
	•	Public key: id_ed25519.pub

⸻

**4.Add SSH Key to GitHub**

	type $HOME.ssh\id_ed25519.pub

Copy the output.

Go to GitHub → Settings → SSH and GPG keys → New SSH key
Paste and save.

⸻

**5.Test SSH Connection**

	ssh -T git@github.com

Type “yes” if prompted.

You should see a message confirming your username.

⸻

**6.Clone the Repository**

	cd $HOME
	git clone git@github.com:Darlock7/MAE155B-Aircraft-Design.git
	cd MAE155B-Aircraft-Design

⸻

**7.Set Git Name and Email**

	git config –global user.name “Your Name”
	git config –global user.email “your_email@example.com”

⸻

MATLAB Setup
	1.	Open the Repository

Set Current Folder in MATLAB to:

C:\Users\YourUserName\MAE155B-Aircraft-Design

⸻
**2.	Run the Project**

run_project
main

Always run run_project before main.

⸻

**MATLAB Git Configuration (CRITICAL)**

Go to: Home → Settings → MATLAB → Source Control → Git

Set the following:
	•	Enable SSH
	•	Uncheck “Use SSH agent”
	•	Public key file:
C:\Users\YourUserName.ssh\id_ed25519.pub
	•	Private key file:
C:\Users\YourUserName.ssh\id_ed25519

If this is not configured correctly, MATLAB push/pull will fail.

⸻

MATLAB GUI Workflow (Primary Method)

Before Starting Work
	1.	Open MATLAB
	2.	Navigate to the repository folder
	3.	Click Source Control → Pull

⸻

If Pull Fails

This usually means local changes would be overwritten.

Fix:
	1.	Open “View Changes”
	2.	Right-click modified file (e.g., main.m)
	3.	Choose:
	•	Discard Changes (to delete local edits)
	•	Commit Changes (to keep them)
	4.	Try Pull again

⸻

While Working
	•	Edit and save files in MATLAB
	•	Avoid renaming/moving files unless necessary

⸻

After Making Changes (Push)
	1.	Open “View Changes”
	2.	Select modified files
	3.	Enter commit message
	4.	Click Commit
	5.	Click Push

⸻

Required Daily Workflow
	1.	Open MATLAB
	2.	Pull latest changes
	3.	Run run_project
	4.	Run main
	5.	Make changes
	6.	Save files
	7.	Commit
	8.	Push

⸻

PowerShell Fallback (If MATLAB Fails)

**If MATLAB fails (push, pull, SSH issues), use:**

	cd C:\Users\YourUserName\MAE155B-Aircraft-Design
	git pull
	git add .
	git commit -m “your message”
	git push

⸻

Force Pull (Overwrite Local Changes)

If you want to completely overwrite your local files:

git fetch origin
git reset –hard origin/main

WARNING: This deletes all local changes permanently.

⸻

Common Problems
	1.	Git not recognized
Fix: reinstall Git and restart PowerShell.
	2.	Permission denied (publickey)
Fix: verify your SSH key:

type $HOME.ssh\id_ed25519.pub

Ensure it matches GitHub.
	3.	MATLAB cannot connect to SSH agent
Fix: disable SSH agent in MATLAB and manually set key paths.
	4.	Push rejected (remote ahead)
Fix:

git pull

Resolve conflicts if needed, then:

git push
	5.	Merge conflicts
Look for markers like:

<<<<<<< HEAD

Fix the file, then:

git add filename
git commit -m “resolve conflict”
git push

⸻

Best Practice
	•	Use MATLAB GUI for pull, commit, and push
	•	Always pull before starting work
	•	Use PowerShell if MATLAB fails
	•	Do not edit files directly on GitHub unless necessary
	•	Do not commit generated files

⸻

Final Note

This workflow ensures MATLAB GUI works reliably on Windows. PowerShell is always the fallback when GUI operations fail.
