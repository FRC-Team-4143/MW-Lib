# assets/launch-tilt-issue

Images and sources referenced by issue #132 (roll/pitch compensation for the LaunchCalculator).
Not code; nothing here is built. Don't merge this branch.

- `launch-tilt/img/` — pictures embedded in the issue
- `launch-tilt/src/sim.py` — point-mass shot sim: miss distance vs robot tilt, and the closed-form compensation check
- `launch-tilt/src/figs.py` — draws the figures (needs `pip install cairosvg`; run from a dir containing `sim.py` and `fig/`)
