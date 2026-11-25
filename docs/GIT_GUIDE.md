# Uploading Version 2 to Your GitHub Repository

A beginner-friendly guide for publishing the new version in the **same** repository (`lokeshkarthi-dev/precision-landing-PX4`) without losing the old one.

---

## The idea in four lines

1. **Keep v1 safe** by giving it a label (a *tag*).
2. **Work on a separate branch** so `main` is never broken while you are changing things.
3. **Copy the new files in, check what changed, save a snapshot (commit), and upload (push).**
4. **Merge** the branch into `main` using a Pull Request, and label the result v2.

### Words you will see

| Word | Plain meaning |
|---|---|
| **Repository (repo)** | The project folder plus its full history |
| **Commit** | A saved snapshot with a message |
| **Branch** | A separate line of work; `main` is the official one |
| **Push** | Upload your commits to GitHub |
| **Pull** | Download new commits from GitHub |
| **Tag** | A permanent label on one commit, such as `v1.0.0` |
| **Pull Request (PR)** | A GitHub page asking to merge your branch into `main` |

---

## Step 0: One-time setup

Check git is installed and tell it who you are (this appears on your commits):

```bash
git --version
git config --global user.name  "Your Name"
git config --global user.email "your-github-email@example.com"
```

**Logging in to GitHub.** GitHub does not accept your account password in the terminal any more. Pick one:

* **GitHub CLI (easiest):** `sudo apt install gh`, then `gh auth login` and follow the prompts.
* **Personal access token:** GitHub, Settings, Developer settings, Personal access tokens. Create one with *repo* permission, and paste it when git asks for a password.
* **SSH key:** more setup; see GitHub's SSH guide.

---

## Step 1: Get a local copy of your repo

```bash
cd ~
git clone https://github.com/lokeshkarthi-dev/precision-landing-PX4.git
cd precision-landing-PX4
git log --oneline -5
```

`git log` should show your latest commit, which should start with `2db8ce5` if it is the version you linked. Check the name of your main branch:

```bash
git branch
```
It shows `main` or `master`. Use that name wherever this guide says `main`.

---

## Step 2: Label v1 so it is never lost

```bash
git tag v1.0.0 2db8ce5
git push origin v1.0.0
```

Now anyone (including you) can get the old version back any time with `git checkout v1.0.0`.

---

## Step 3: Create a branch for v2

```bash
git checkout -b v2-monocam
git branch
```
The `*` shows you are on `v2-monocam`.

---

## Step 4: Copy the new files in

Unzip or download the `precision-landing-v2` folder (for example into `~/Downloads`). Then copy **everything inside it** into your repo folder:

```bash
cp -r ~/Downloads/precision-landing-v2/. ~/precision-landing-PX4/
```
The `/.` at the end also copies hidden files such as `.gitignore`.

### What happens to each file

| File or folder | Action | Notes |
|---|---|---|
| `src/camera_node.py` | **Replace** | New version |
| `src/precision_landing.py` | **Replace** | New version |
| `config/params.yaml` | **Replace** | New settings (your old one is replaced; the old values are listed in the changelog) |
| `README.md` | **Replace** | Rewritten for v2 |
| `requirements.txt` | **Replace** | Updated versions |
| `.gitignore` | **Add** | Stops junk files from being uploaded |
| `docs/SITL_RUNBOOK.md`, `CODE_AUDIT.md`, `CHANGELOG.md`, `GIT_GUIDE.md` | **Add** | New documentation |
| `assets/` | **Leave alone** | Marker generator is unchanged |
| `tests/test_aruco_detection.py` | **Leave alone** | Run it again after copying (Step 5) |
| `Precision_Landing_for_Multirotors.pdf` | **Move** (see below) | It describes v1 |

### Handle the old PDF

The PDF explains v1 (the old topics and constants). Move it into a `docs/v1/` folder so nobody mistakes it for the current design:

```bash
mkdir -p docs/v1
git mv Precision_Landing_for_Multirotors.pdf docs/v1/
```

You can regenerate a v2 PDF later from the Markdown files.

---

## Step 5: Check before you save

```bash
python3 tests/test_aruco_detection.py     # existing offline tests still pass?
python3 -m py_compile src/camera_node.py src/precision_landing.py   # no syntax errors
git status
```

`git status` lists what changed:

* *modified* = existing files you replaced (expected for `src/`, `config/`, `README.md`, `requirements.txt`)
* *new / untracked* = new files (`docs/...`, `.gitignore`)
* *renamed* = the PDF moved

To see exactly what changed in one file:
```bash
git diff src/precision_landing.py
```
(Press `q` to leave the viewer.)

**Make sure you do not see** `__pycache__/`, `build/`, or any large files. If you do, check that `.gitignore` was copied.

---

## Step 6: Commit (save a snapshot)

A tidy history is easier to read. Two or three commits is better than one huge one:

```bash
git add src/camera_node.py src/precision_landing.py config/params.yaml
git commit -m "Switch to downward monocam; safer controller with marker-loss handling"

git add docs/ .gitignore requirements.txt
git commit -m "Add runbook, code audit, changelog and Git guide"

git add README.md
git commit -m "Rewrite README for v2"
```

Check the result:
```bash
git log --oneline -5
git status        # should say "nothing to commit, working tree clean"
```

---

## Step 7: Push the branch to GitHub

```bash
git push -u origin v2-monocam
```
(`-u` links your local branch to the GitHub one, so next time `git push` is enough.)

---

## Step 8: Open a Pull Request and merge

1. Open your repo on GitHub. A yellow banner says *"v2-monocam had recent pushes"*. Click **Compare & pull request**.
2. Title: `v2: downward monocam and safer controller`.
3. In the description, paste the "What changed and what improved" section from `docs/CHANGELOG.md`.
4. Click **Create pull request**, read the *Files changed* tab once, then click **Merge pull request** and **Confirm**.

Bring the merged `main` back to your computer:
```bash
git checkout main
git pull
```

---

## Step 9: Tag v2 and publish a release

```bash
git tag v2.0.0
git push origin v2.0.0
```

On GitHub: **Releases**, **Draft a new release**, choose tag `v2.0.0`, paste the changelog text and publish.

---

## If something goes wrong

| Problem | What to do |
|---|---|
| Typo in the last commit message | `git commit --amend -m "new message"` (only before pushing) |
| Edited a file by mistake, want it back | `git restore path/to/file` |
| Added a file you did not want to commit | `git restore --staged path/to/file` |
| `git push` rejected ("fetch first") | `git pull --rebase`, then `git push` |
| `Permission denied` or login fails | Redo the login step in Step 0 (token or `gh auth login`) |
| Pushed something secret by mistake | Do not just delete it; change the secret (password or token) first, then ask for help removing it from history |
| Want to see the old version | `git checkout v1.0.0` (go back with `git checkout main`) |
| Totally lost | Copy your folder somewhere as a backup, then ask for help with the output of `git status` and `git log --oneline -5` |

---

## Simple alternative: upload through the web page

If the terminal feels like too much, you can use GitHub's website: open the repo, click **Add file**, **Upload files**, drag the new files in and commit. Be aware that:

* it does not create a branch or tag unless you choose to,
* it cannot move files (like the PDF) as neatly,
* it is easy to forget a file.

It works, but learning the steps above will help you in any future project or team.

---

## Habits that help

* Commit small and often, with messages that say **what** and **why**.
* Run `git status` before every commit.
* Never commit passwords, tokens or large generated files.
* Keep `main` working; do new work on a branch.
