# How to use your Interceptor Sim (plain-English guide)

Everything is installed. You just use the launcher.

## Every time: one click
Double-click **"Launch Interceptor Sim (WSL)"** on your Desktop.
- It opens a terminal straight into your Linux environment (WSL) and starts Claude Code in the project.
- Normal terminal window - copy/paste works (Ctrl+Shift+V to paste).
- Your **RTX 4070 GPU** is available inside it.

## The very first time only (about a minute)
Claude Code walks you through it:
1. Pick a color theme - press **Enter** for the default.
2. Type **/login** - open the link, sign in, and paste the code back.
3. Type **/model** and pick **Fable 5**. (Helper subagents auto-use Sonnet 5.)
4. Type **/go** and press **Enter**. That runs your whole kickoff.

After that, the launcher drops you **straight back into your session**.

## What is in the project
In WSL at `~/interceptor-sim` (Windows mirror in your Downloads > Simulation Work > interceptor-sim folder):
- **GOALS.md** - what you are building and why.
- **CLAUDE.md** - how Claude works, the model + council setup, your setup, and who it is helping (you).
- **KICKOFF-PROMPT.md** - the full kickoff (the /go command runs this).
- **.claude/skills/** - px4-gazebo, pronav, sim-milestone, sim-debug (used automatically). Built-in /code-review and /debug too.
- **.claude/agents/** - the Sonnet worker, decision council, and verifier.

## Handy things to know
- **Reopen / resume:** re-run the launcher.
- **GPU:** run `nvidia-smi` in the terminal to see your RTX 4070. Gazebo can use it; batch runs still go headless for speed.
- **Linux login** (rarely needed): user **emerson**, password **interceptor123**.
- If a Gazebo window will not render, that is WSL OpenGL being finicky - Claude falls back to headless.
